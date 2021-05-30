import math
import numpy as np

from rostron_interfaces.action import MoveTo
from rostron_utils.world import World

# TODO(Etienne) Chercher un nom pour la classe mère
class AStar():

    def __init__(self, id: int, goal : MoveTo.Goal) -> None:
        resolution = 0.1
        margin = 3*2
        self.x_ = math.ceil(World().field.width / resolution) + margin
        self.y_ = math.ceil(World().field.length / resolution) + margin

        self.id_ = id
        self.goal_ = goal
    
    def create_grid(self):
        grid = np.zeros((self.y_, self.x_))

        """
        1. Parcourir l'ensemble des alliées et des opposants.
        Durée : Une demi-journée maximum
           
        Ajouter ces obstacles au niveau de la grille crée en haut.
        0 -> libre
        2 -> obstacle (impossible d'y passer)
        
        Il faut convertir les coordonées du robot, dans la grille.
        Si tu ne comprends pas comment faire, je te conseille de poser sur un dessin.
        En y réfléchissant j'ai vu une translation puis un -y.
        (x_robot, y_robot) -> (x_grid, y_grid)
        Cette transformation doit être facilement changeable en fonction des données du terrain (pas de valeur en dure !)

        /!\ Si l'id de l'allié est le même, ... (je te laisse deviner)
        /!\ J'ai ajouté des marges en x et en y

        2. Convertir la grille numpy en une map sous ROS affichable sous RVIZ via un topic map?.
        Durée : Une demi-journée 
        Je te laisse chercher le type, etc....
        Y a un type pour ça dans common_msgs de ROS2.
        """

    def run(self):
        self.create_grid()