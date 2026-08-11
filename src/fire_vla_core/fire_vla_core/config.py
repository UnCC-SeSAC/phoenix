from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(slots=True)
class AppConfig:
    raw: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "AppConfig":
        with Path(path).open("r", encoding="utf-8") as f:
            return cls(yaml.safe_load(f) or {})

    def get(self, dotted: str, default: Any = None) -> Any:
        value: Any = self.raw
        for key in dotted.split("."):
            if not isinstance(value, dict) or key not in value:
                return default
            value = value[key]
        return value
