class SquareMap():
    #Initialise and create an empty map with (x,y) dimensions
    def __init__(self, x: int, y: int) -> None:
        super().__init__()
        self.map = [[0 for i in range(x)] for j in range(y)]

    def set_xy_map(self,pos: tuple,value: int) -> None:
        if len(self.map[-1])>pos[1] and len(self.map)>pos[0]:
            self.map[pos[0]][pos[1]]=value
        else :
            print("ERROR : Position out of the map  ", pos)