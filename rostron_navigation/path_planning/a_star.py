import numpy

# TODO Chercher un nom pour la classe mère
class AStar():

    def __init__(self) -> None:
        self.create_grid()
        pass
    
    def create_grid(self):
        # keep field 
        field_length = 10.4
        field_width = 7.4
        resolution = 0.1

        grid_x = field_width / resolution
        grid_y = field_length / resolution
        # np.zeros((grid_x, grid_y))
        numpy.zeros((grid_x, grid_y))
        pass
