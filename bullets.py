"""
.338 Norma Magnum bullet library.

BC and velocity values are representative published/typical figures and MUST be
verified against your load and, ideally, trued to observed drop. G7 BCs vary by
source (Berger/Litz measured vs advertised) and velocity band. Treat these as
sensible defaults, not gospel -- every field in the UI is editable.

Diameter is the bullet caliber (0.338"). Lengths are nominal.
"""

from .solver import Bullet

# name -> (mass_gr, diameter_in, length_in, g7_bc, typical_mv_fps)
_LIBRARY = {
    "Berger 300gr Hybrid OTM": Bullet("Berger 300gr Hybrid OTM", 300, 0.338, 1.800, 0.379),
    "Berger 300gr Elite Hunter": Bullet("Berger 300gr Elite Hunter", 300, 0.338, 1.790, 0.368),
    "Sierra 300gr MatchKing": Bullet("Sierra 300gr MatchKing", 300, 0.338, 1.780, 0.360),
    "Hornady 285gr ELD-M": Bullet("Hornady 285gr ELD-M", 285, 0.338, 1.712, 0.345),
    "Lapua 300gr Scenar-L": Bullet("Lapua 300gr Scenar-L", 300, 0.338, 1.775, 0.331),
    "Berger 250gr Elite Hunter": Bullet("Berger 250gr Elite Hunter", 250, 0.338, 1.560, 0.313),
    "Lapua 250gr Scenar": Bullet("Lapua 250gr Scenar", 250, 0.338, 1.548, 0.303),
}

# Typical muzzle velocities in a .338 NM (fps), 26-27" barrel, book-ish loads.
TYPICAL_MV = {
    "Berger 300gr Hybrid OTM": 2750,
    "Berger 300gr Elite Hunter": 2760,
    "Sierra 300gr MatchKing": 2740,
    "Hornady 285gr ELD-M": 2790,
    "Lapua 300gr Scenar-L": 2745,
    "Berger 250gr Elite Hunter": 2960,
    "Lapua 250gr Scenar": 2950,
}


def list_bullets():
    return [
        {
            "name": b.name,
            "mass_grains": b.mass_grains,
            "diameter_in": b.diameter_in,
            "length_in": b.length_in,
            "bc_g7": b.bc_g7,
            "twist_in": b.twist_in,
            "typical_mv_fps": TYPICAL_MV.get(name, 2750),
        }
        for name, b in _LIBRARY.items()
    ]


def get_bullet(name: str) -> Bullet:
    return _LIBRARY[name]
