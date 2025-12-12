#!/usr/bin/env python3
import time

class UltrasonicArray:
    """Classe Python pour représenter les données des capteurs ultrasoniques
    Équivalent au message ROS2 personnalisé UltrasonicArray.msg
    """
    
    def __init__(self, distances=None, timestamp=None):
        self.distances = distances or [0.0, 0.0, 0.0, 0.0]  # 4 capteurs
        self.timestamp = timestamp or time.time()  # timestamp en secondes
    
    def __str__(self):
        return f"UltrasonicArray(distances={self.distances}, timestamp={self.timestamp:.2f})"
    
    def to_dict(self):
        """Convertit en dictionnaire pour la sérialisation"""
        return {
            'distances': self.distances,
            'timestamp': self.timestamp
        }
    
    @classmethod
    def from_dict(cls, data):
        """Crée une instance à partir d'un dictionnaire"""
        return cls(
            distances=data.get('distances', [0.0, 0.0, 0.0, 0.0]),
            timestamp=data.get('timestamp', time.time())
        )
    
    def get_min_distance(self):
        """Retourne la distance minimale détectée"""
        return min(self.distances)