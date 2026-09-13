"""Which velocity source drives the car right now.

Pure Python, no ROS: the node feeds messages in with their arrival time and
asks which source, if any, is fresh and highest priority. A source that has
not spoken within its timeout drops out. No fresh source means stop.
"""
from dataclasses import dataclass
from typing import Any, Dict, Iterable, NamedTuple, Optional, Tuple


@dataclass(frozen=True)
class Source:
    name: str
    priority: int        # higher wins; must be distinct across sources
    timeout_s: float     # silence longer than this drops the source out


class Selection(NamedTuple):
    source: str
    command: Any


class MuxPolicy:
    def __init__(self, sources: Iterable[Source]):
        sources = list(sources)
        if not sources:
            raise ValueError('a velocity mux needs at least one source')
        if len({s.priority for s in sources}) != len(sources):
            raise ValueError('source priorities must be distinct')
        for s in sources:
            if s.timeout_s <= 0.0:
                raise ValueError(f'source {s.name!r} needs a positive timeout')
        self._by_priority = sorted(sources, key=lambda s: s.priority, reverse=True)
        self._latest: Dict[str, Tuple[float, Any]] = {}

    @property
    def sources(self) -> Tuple[Source, ...]:
        return tuple(self._by_priority)

    def offer(self, source: str, command: Any, now: float) -> None:
        """Record the latest command from a source. Raises KeyError for an unknown source."""
        if source not in {s.name for s in self._by_priority}:
            raise KeyError(source)
        self._latest[source] = (now, command)

    def select(self, now: float) -> Optional[Selection]:
        """The highest-priority source that spoke within its timeout, or None."""
        for s in self._by_priority:
            entry = self._latest.get(s.name)
            if entry is None:
                continue
            stamp, command = entry
            if now - stamp <= s.timeout_s:
                return Selection(s.name, command)
        return None
