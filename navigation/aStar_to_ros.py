import math
"""
    E.S :
    On n'importe pas d'autres projets comme ça. 
    Si tu veux qu'on est un endroit où partagé du code on le fait via une librairie.
    rostron_ia_ms est un repositories qui doit être facilement enlevable.
"""
from rostron_ia_ms.utils.world import World

"""
    E.S : Une classe serait mieux et permettrait de créer une grille de x * y.
    Proposer un constructeur peut-être en fonction de la résolution.
"""
# Create a map of SSL
SquareMap = [[0 for x in range(98)] for y in range(68)] 

"""
    Je pense que le commentaire ne va pas avec la fonction.
"""
# Create squares with poses (square = 10cm²)
def pose_to_square(x, y):
    return (math.floor(10*(x + 4.9)),math.floor(10 * ( y + 3.4)))

def square_to_pose(x, y):
    poseX= -4.9+0.1*x+0.05
    poseY= -3.4+0.1*y+0.05
    return (round(poseX, 1),round(poseY, 1))

# if robot is in a square : square = -1
def RobotisInSquare(square):
    radius=0.085
    distRobot= 6+radius
    pose= square_to_pose(square)
    # Tu peux faire une itération sur le tableau alliés.
    for i in range(6):
        x = World().allies[i].pose.position.x
        y = World().allies[i].pose.position.y
        xO = World().allies[i].pose.position.x
        yO = World().allies[i].pose.position.y
        # Essaye de passer par des utilisation via numpy.
        if distance(x,y,pose[0],pose[1])<distRobot or distance(xO,yO,pose[0],pose[1])<distRobot : 
            if i!=0:
                SquareMap[square[0]][square[1]]= -1  #-1 for obstacles
            elif distance(x,y,pose[0],pose[1])>distRobot: #if robot not yellow and id=0
                SquareMap[square[0]][square[1]]= -1 


"""
Inutile déjà implémenter dans numpy.
"""
def distance (x, y, xa, ya):
  return math.sqrt(((xa-x)**2)+((ya-y)**2))



"""
A quelle moment on utilise ROS ? 
Il n'y a aucun noeud crée !
Si tu veux faire un main de test, je te conseille de faire un truc de ce genre

if __name__ == "__main__":
    # execute only if run as a script
    ...

Voir https://docs.python.org/fr/3/library/__main__.html
"""
#aStar with the robot 0
def main(args=None):
    #BeginSquare with the current position of robot 0
    s = pose_to_square(World().allies[0].pose.position.x, World().allies[0].pose.position.y)
    SquareMap[s[0]][s[1]]=1
    
    #Square Arrival
    SquareMap[32][21]=2

    #All obstacles
    for i in range(len(SquareMap)):
        for j in range(len(SquareMap[i])):
            RobotisInSquare(SquareMap[i][j])

    # Create the path to SquareArrival
    path = a_star(SquareMap)

    #Transform path with squares in path with poses
    for i in range(len(path)) : 
        path[i]= square_to_pose(path[i])

    # There is a moveTo to every poses of the path
    # The robot will go to every poses(squares) one by one (without interuption)
    moveTo(path)
    
    