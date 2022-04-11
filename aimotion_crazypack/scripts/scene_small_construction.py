import pickle
from planner_tools import *
#----------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    scene = Scene()                                  # Scene object is defined in the planner_tools.
                                                         # We use it to store base data.
    scene.dims = np.array([-5, 5, -5, 5])            # The size of the scene
    scene.fobs = np.array([[-3, -3, 2], [3, -3, 2],  # Position and size of the fix obstacles
                           [-3, 3, 2],  [3, 3, 2]])
    scene.box = np.array([[(-1.5*0.5, 0), (0, -1.5*0.5), (1.5*0.5, 0), (0, 1.5*0.5)],
                          [(-1*0.5, -1*0.5), (1*0.5, -1*0.5), (1*0.5, 1*0.5), (-1*0.5, 1*0.5)]])        # xy koordinates of the rectangle corners
# ----------------------------------------------------------------------------------------------------------------------
    try:                                             # Open the xy_list if it exists
        pickle_in = open("xy_file.pickle", "rb")
        xy_list = pickle.load(pickle_in)
        #scene.mobs = xy_list                        # Show all existing paths if uncommented
    except (OSError, IOError) as e:                  # Create an empty xy_list
        xy_list = []
# ----------------------------------------------------------------------------------------------------------------------
    create_view(scene, [])                           # Draw the fobs (and mob paths if uncommented above).
    xy_list += get_path(xy_list)                     # Add new mob paths
# ----------------------------------------------------------------------------------------------------------------------
    pickle_out = open("scene_file.pickle", "wb")     # Save scene
    pickle.dump(scene, pickle_out)
    pickle_out.close()

    pickle_out = open("xy_file.pickle", "wb")        # Save xy_list
    pickle.dump(xy_list, pickle_out)
    pickle_out.close()
