from __future__ import annotations

import dataclasses

from object_profiling.config import AppConfig, DamageConfig


def intact_config(**overrides) -> AppConfig:
    """Configuracion sin dano, para no mezclar precision dimensional con CP1."""

    base = AppConfig()
    return dataclasses.replace(base, damage=dataclasses.replace(base.damage, rate=0.0), **overrides)


def damaged_config(rate: float = 0.5, **overrides) -> AppConfig:
    base = AppConfig()
    return dataclasses.replace(base, damage=dataclasses.replace(base.damage, rate=rate), **overrides)
