import logging.config
import warnings
from typing import Any


LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "standard": {"format": "%(asctime)s [%(levelname)s] %(name)s: [%(funcName)s] %(message)s"},
    },
    "handlers": {
        "console": {
            "level": "ERROR",
            "class": "logging.StreamHandler",
            "formatter": "standard",
        },
        "file": {
            "level": "DEBUG",
            "class": "logging.FileHandler",
            "formatter": "standard",
            "filename": "trano.log",
            "mode": "w+",
        },
    },
    "loggers": {
        "": {
            "handlers": ["console", "file"],
            "level": "DEBUG",
            "propagate": True,
        },
    },
}


logging.config.dictConfig(LOGGING_CONFIG)
logging.getLogger("docker").setLevel(logging.CRITICAL)
logging.getLogger("urllib3").setLevel(logging.CRITICAL)
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings(
    "ignore",
    category=UserWarning,
)


__all__ = ["check_yaml", "simulate_yaml"]


def __getattr__(name: str) -> Any:  # noqa: ANN401
    """The one-call API, imported on first use: importing ``trano`` stays cheap."""
    if name in __all__:
        from trano import api

        return getattr(api, name)
    raise AttributeError(f"module 'trano' has no attribute {name!r}")
