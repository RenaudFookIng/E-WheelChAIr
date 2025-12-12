#!/usr/bin/env python3

class EmergencyData:
    """Classe Python pour représenter les données d'urgence
    Équivalent au message ROS2 personnalisé EmergencyData.msg
    """
    
    def __init__(self, obstacle_detected=False, distance=0.0, angle=0.0, source="unknown"):
        self.obstacle_detected = obstacle_detected  # bool
        self.distance = distance  # float32: distance en mètres
        self.angle = angle  # float32: angle en radians
        self.source = source  # string: "lidar" ou "depth_camera"
    
    def __str__(self):
        return f"EmergencyData(detected={self.obstacle_detected}, dist={self.distance:.2f}m, angle={self.angle:.2f}rad, source={self.source})"
    
    def to_dict(self):
        """Convertit en dictionnaire pour la sérialisation"""
        return {
            'obstacle_detected': self.obstacle_detected,
            'distance': self.distance,
            'angle': self.angle,
            'source': self.source
        }
    
    @classmethod
    def from_dict(cls, data):
        """Crée une instance à partir d'un dictionnaire"""
        return cls(
            obstacle_detected=data.get('obstacle_detected', False),
            distance=data.get('distance', 0.0),
            angle=data.get('angle', 0.0),
            source=data.get('source', 'unknown')
        )