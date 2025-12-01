import octomap
import numpy as np

# --- 1. PARAMÈTRES DU MONDE (Basés sur hybrid_map.world) ---
RESOLUTION = 0.1  # Résolution du voxel (10 cm)
WORLD_X = 15.0  
WORLD_Y = 10.0
WALL_THICKNESS = 0.2
WALL_HEIGHT = 3.0

# Initialiser l'OctoMap avec la résolution
tree = octomap.OcTree(RESOLUTION)

# Définir le centre de la hauteur du mur
Z_START = 0.0
Z_END = WALL_HEIGHT

def fill_box(min_x, max_x, min_y, max_y, min_z, max_z):
    """Marque tous les voxels dans la boîte spécifiée comme occupés."""
    # Itérer sur l'espace 3D avec le pas de la résolution
    for x in np.arange(min_x, max_x, RESOLUTION):
        for y in np.arange(min_y, max_y, RESOLUTION):
            for z in np.arange(min_z, max_z, RESOLUTION):
                tree.updateNode(x, y, z, True) # True = Occupé

# --- 2. DÉFINITION DE LA GÉOMÉTRIE (Basé sur les coordonnées du monde) ---

# MURS D'ENCEINTE (épaisseur de 0.2m)
# Mur Est (x=15)
fill_box(WORLD_X - WALL_THICKNESS, WORLD_X, 0, WORLD_Y, Z_START, Z_END)
# Mur Ouest - Mur Sud (de y=0 à y=4)
fill_box(0, WALL_THICKNESS, 0, 4.0, Z_START, Z_END)
# Mur Ouest - Mur Nord (de y=6 à y=10) - L'ouverture est entre 4 et 6
fill_box(0, WALL_THICKNESS, 6.0, WORLD_Y, Z_START, Z_END)
# Mur Nord (y=10)
fill_box(0, WORLD_X, WORLD_Y - WALL_THICKNESS, WORLD_Y, Z_START, Z_END)
# Mur Sud (y=0)
fill_box(0, WORLD_X, 0, WALL_THICKNESS, Z_START, Z_END)

# OBSTACLES INTERNES (Hauteur 1.0m, centré)
# Obstacle 1 (centré en 4, 3)
fill_box(3.5, 4.5, 1.0, 5.0, Z_START, 1.0) 
# Obstacle 2 (centré en 7, 7)
fill_box(6.5, 7.5, 5.0, 9.0, Z_START, 1.0) 

# MUR FINAL DE BLOCAGE (Bleu, épaisseur 0.5m, hauteur 3.0m, centré à x=10)
fill_box(9.75, 10.25, 0, WORLD_Y, Z_START, WALL_HEIGHT) 

# --- 3. SAUVEGARDE DE LA CARTE ---
output_path = "../map/storage_level_0.bt" # Enregistre dans le dossier map
tree.writeBinary(output_path)
print(f"OctoMap statique généré et enregistré dans : {output_path}")
