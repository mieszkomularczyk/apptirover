"""Telemetry snapshots distinguish an open port, echoes, and fresh sensor data."""

import copy
from dataclasses import dataclass, field

from .protocol import QUERY_TYPES, TELEMETRY_TYPES


@dataclass
class Telemetry:
    port: str
    baud: int = 115200
    connected: bool = False
    generation: int = 0
    opened_at: float | None = None
    tx_queries: int = 0
    rx_bytes: int = 0
    echoes: int = 0
    received: int = 0
    invalid_lines: int = 0
    reconnects: int = 0
    last_error: str = ""
    packets: dict = field(default_factory=dict)
    other: dict = field(default_factory=dict)

    def opened(self, now):
        if self.generation:
            self.reconnects += 1
        self.generation += 1
        self.connected = True
        self.opened_at = now
        self.last_error = ""

    def accept(self, message, now):
        kind = message["T"]
        if kind in QUERY_TYPES:
            self.echoes += 1
            return
        sample = {"data": dict(message), "received_at": now, "generation": self.generation}
        # A type marker alone is not evidence of usable telemetry.
        if kind in TELEMETRY_TYPES and any(
            type(message.get(key)) in (int, float)
            for key in ("v", "L", "R", "r", "p", "y", "ax", "gx", "mx", "temp")
        ):
            self.received += 1
            self.packets[kind] = sample
        else:
            # Preserve new firmware reply types without unbounded accumulation.
            if kind not in self.other and len(self.other) >= 16:
                del self.other[next(iter(self.other))]
            self.other[kind] = sample

    def snapshot(self, now, stale_after=3.0):
        packets = {}
        for kind, sample in self.packets.items():
            age = max(0.0, now - sample["received_at"])
            packets[str(kind)] = {
                "data": copy.deepcopy(sample["data"]),
                "age_s": round(age, 3),
                "fresh": self.connected and sample["generation"] == self.generation and age <= stale_after,
            }
        fresh = {int(kind) for kind, value in packets.items() if value["fresh"]}
        if not self.connected:
            state = "disconnected"
        elif fresh == TELEMETRY_TYPES:
            state = "telemetry live"
        elif fresh:
            state = "partial telemetry"
        elif packets:
            state = "telemetry stale"
        else:
            state = "waiting for telemetry"
        return {
            "port": self.port, "baud": self.baud, "port_open": self.connected,
            "state": state, "generation": self.generation, "reconnects": self.reconnects,
            "tx_queries": self.tx_queries, "rx_bytes": self.rx_bytes,
            "telemetry_packets": self.received, "echoes": self.echoes,
            "invalid_lines": self.invalid_lines, "last_error": self.last_error,
            "packets": packets,
            "other_packets": {
                str(kind): {
                    "data": copy.deepcopy(sample["data"]),
                    "age_s": round(max(0, now - sample["received_at"]), 3),
                    "fresh": self.connected and sample["generation"] == self.generation
                    and now - sample["received_at"] <= stale_after,
                } for kind, sample in self.other.items()
            },
        }
