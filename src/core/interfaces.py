from abc import ABC, abstractmethod
from typing import Dict, Any
import numpy as np

class CenterlineGenerator(ABC):
    def __init__(self, config: Dict[str, Any], seed: int = None):
        self.config = config
        self.seed = seed
        if seed is not None:
            np.random.seed(seed)

    @abstractmethod
    def generate(self) -> 'CenterlineData':
        """
        Generate a centerline.
        Returns:
            CenterlineData: object containing points (x,y), curvature (kappa), arc length (s)
        """
        pass

class TrackGenerator(ABC):
    def __init__(self, config: Dict[str, Any], seed: int = None):
        self.config = config
        self.seed = seed

    @abstractmethod
    def generate(self, centerline: 'CenterlineData') -> 'TrackData':
        """
        Generate track boundaries from a given centerline.
        Args:
            centerline: CenterlineData from previous step.
        Returns:
            TrackData: object containing left and right boundary points, width profile, etc.
        """
        pass

class AirDuctPlacer(ABC):
    def __init__(self, config: Dict[str, Any], seed: int = None):
        self.config = config
        self.seed = seed

    @abstractmethod
    def place(self, track: 'TrackData') -> 'AirDuctsData':
        """
        Place air ducts along the track.
        Args:
            track: TrackData from previous step.
        Returns:
            AirDuctsData: object containing OBBs or probabilistic placement info.
        """
        pass