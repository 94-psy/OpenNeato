#!/usr/bin/env python3
import math
from typing import List, Tuple
from geometry_msgs.msg import PoseStamped

class ZoneCoveragePlanner:
    """
    Gestisce la pianificazione del percorso di copertura (Coverage Path Planning)
    per le zone di pulizia utilizzando un algoritmo Boustrophedon (a serpentina).
    """
    def __init__(self, stride: float = 0.25):
        """
        Inizializza il pianificatore.
        
        Args:
            stride (float): Distanza tra le linee parallele (metri). 
                            Default 0.25m (metà larghezza robot tipico).
        """
        self.stride = stride

    def is_point_in_polygon(self, x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
        """
        Verifica se il punto (x,y) è interno al poligono usando Ray Casting.
        """
        inside = False
        j = len(polygon) - 1
        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            
            intersect = ((yi > y) != (yj > y)) and \
                        (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            if intersect:
                inside = not inside
            j = i
        return inside

    def generate_boustrophedon_path(self, zone_points: List[Tuple[float, float]]) -> List[PoseStamped]:
        """
        Genera una lista di waypoint a zig-zag filtrando i punti esterni al poligono.
        
        Args:
            zone_points: Lista di tuple (x, y) che definiscono i vertici della zona.
            
        Returns:
            List[PoseStamped]: Lista ordinata di waypoint per il navigatore.
        """
        if not zone_points:
            return []

        # 1. Calcolo Bounding Box
        x_coords = [p[0] for p in zone_points]
        y_coords = [p[1] for p in zone_points]
        
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        waypoints = []
        y = y_min
        row_count = 0
        
        # 2. Generazione percorso a griglia filtrata (Grid-based Boustrophedon)
        while y <= y_max:
            row_waypoints = []
            x = x_min
            
            # Scansiona la riga X
            while x <= x_max:
                if self.is_point_in_polygon(x, y, zone_points):
                    p = PoseStamped()
                    p.header.frame_id = "map"
                    p.pose.position.x = x
                    p.pose.position.y = y
                    p.pose.position.z = 0.0
                    p.pose.orientation.w = 1.0
                    row_waypoints.append(p)
                x += self.stride

            # Se abbiamo trovato punti validi in questa riga
            if row_waypoints:
                # Se riga dispari, inverti ordine (zig-zag)
                if row_count % 2 != 0:
                    row_waypoints.reverse()
                waypoints.extend(row_waypoints)
            
            y += self.stride
            row_count += 1
            
        return waypoints