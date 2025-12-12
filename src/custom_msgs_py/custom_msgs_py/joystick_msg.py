#!/usr/bin/env python3

class Joystick:
    """Classe Python pour représenter les données de joystick
    Équivalent au message ROS2 personnalisé Joystick.msg
    """
    
    def __init__(self, x=0.0, y=0.0):
        self.x = x  # float32: axe X (-1.0 à 1.0)
        self.y = y  # float32: axe Y (-1.0 à 1.0)
    
    def __str__(self):
        return f"Joystick(x={self.x:.2f}, y={self.y:.2f})"
    
    def to_dict(self):
        """Convertit en dictionnaire pour la sérialisation"""
        return {'x': self.x, 'y': self.y}
    
    @classmethod
    def from_dict(cls, data):
        """Crée une instance à partir d'un dictionnaire"""
        return cls(x=data.get('x', 0.0), y=data.get('y', 0.0))