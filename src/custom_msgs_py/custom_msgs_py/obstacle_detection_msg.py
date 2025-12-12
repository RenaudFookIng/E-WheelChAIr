#!/usr/bin/env python3

class ObstacleDetection:
    """Classe Python pour représenter les données de détection d'obstacles
    Équivalent au message ROS2 personnalisé ObstacleDetection.msg
    """
    
    def __init__(self, id=0, obstacle_class="unknown", confidence=0.0, 
                 bbox=None, relative_distance=0.0, offset_from_center=0.0):
        self.id = id  # int32: identifiant de l'obstacle
        self.obstacle_class = obstacle_class  # string: classe de l'obstacle
        self.confidence = confidence  # float32: confiance (0.0-1.0)
        self.bbox = bbox or [0, 0, 0, 0]  # int32[4]: [x, y, width, height]
        self.relative_distance = relative_distance  # float32: distance relative
        self.offset_from_center = offset_from_center  # float32: décalage par rapport au centre
    
    def __str__(self):
        return f"ObstacleDetection(class={self.obstacle_class}, conf={self.confidence:.2f}, dist={self.relative_distance:.2f}m)"
    
    def to_dict(self):
        """Convertit en dictionnaire pour la sérialisation"""
        return {
            'id': self.id,
            'obstacle_class': self.obstacle_class,
            'confidence': self.confidence,
            'bbox': self.bbox,
            'relative_distance': self.relative_distance,
            'offset_from_center': self.offset_from_center
        }
    
    @classmethod
    def from_dict(cls, data):
        """Crée une instance à partir d'un dictionnaire"""
        return cls(
            id=data.get('id', 0),
            obstacle_class=data.get('obstacle_class', 'unknown'),
            confidence=data.get('confidence', 0.0),
            bbox=data.get('bbox', [0, 0, 0, 0]),
            relative_distance=data.get('relative_distance', 0.0),
            offset_from_center=data.get('offset_from_center', 0.0)
        )