"""
Modbus TCP transport, isolated from everything that knows about JVL.

Two reasons this is its own layer:

1. pymodbus keeps renaming things. The keyword naming the target device has
   been `unit=` (pymodbus 2.x), `slave=` (3.0-3.9) and `device_id=` (3.10+ and
   4.x), and `count` has moved between positional and keyword-only. Rather
   than sprinkle try/except around every call site, the naming is worked out
   once, at construction, by inspecting the installed client's signature.

2. It lets the simulator substitute for a real motor at exactly the boundary
   where the wire is, so every line of JVL logic above this file is exercised
   identically whether or not hardware is present.
"""

from __future__ import annotations

import inspect
import threading
from typing import List, Optional, Protocol


class ModbusError(IOError):
    """Any failure to complete a Modbus transaction."""


class Transport(Protocol):
    """The minimum a motor needs from whatever is carrying its registers."""

    def connect(self) -> bool: ...
    def close(self) -> None: ...
    def is_open(self) -> bool: ...
    def read_holding(self, address: int, count: int) -> List[int]: ...
    def write_holding(self, address: int, values: List[int]) -> None: ...
    def describe(self) -> str: ...


class PymodbusTransport:
    """Modbus TCP over pymodbus, version differences absorbed."""

    def __init__(self, host: str, port: int = 502, unit_id: int = 1,
                 timeout_s: float = 2.0):
        self.host = host
        self.port = int(port)
        self.unit_id = int(unit_id)
        self.timeout_s = float(timeout_s)
        self._client = None
        self._unit_kw: Optional[str] = None
        self._count_is_kw: bool = True
        self._lock = threading.RLock()

    # ---------------------------------------------------------------- setup

    def _build_client(self):
        try:
            from pymodbus.client import ModbusTcpClient  # pymodbus >= 3
        except ImportError:  # pragma: no cover - very old pymodbus
            try:
                from pymodbus.client.sync import ModbusTcpClient  # type: ignore
            except ImportError as exc:
                raise ModbusError(
                    "pymodbus is not installed. Run:  pip install pymodbus"
                ) from exc

        # `timeout` has been accepted by every 2.x/3.x/4.x constructor, but
        # guard anyway so a future rename degrades to the default rather than
        # refusing to connect at all.
        try:
            return ModbusTcpClient(self.host, port=self.port, timeout=self.timeout_s)
        except TypeError:
            return ModbusTcpClient(self.host, port=self.port)

    def _detect_call_convention(self) -> None:
        """Work out this pymodbus version's keyword names, once."""
        sig = inspect.signature(self._client.read_holding_registers)
        params = sig.parameters
        for candidate in ("device_id", "slave", "unit"):
            if candidate in params:
                self._unit_kw = candidate
                break
        else:
            # Nothing recognisable: fall through and let the default device id
            # apply. Most JVL setups answer on any unit id over TCP anyway.
            self._unit_kw = None

        count_param = params.get("count")
        self._count_is_kw = (
            count_param is None
            or count_param.kind is not inspect.Parameter.POSITIONAL_ONLY
        )

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> bool:
        with self._lock:
            if self._client is None:
                self._client = self._build_client()
                self._detect_call_convention()
            ok = bool(self._client.connect())
            return ok

    def close(self) -> None:
        with self._lock:
            if self._client is not None:
                try:
                    self._client.close()
                except Exception:
                    pass

    def is_open(self) -> bool:
        with self._lock:
            client = self._client
            if client is None:
                return False
            # pymodbus exposes connectedness differently across versions.
            connected = getattr(client, "connected", None)
            if isinstance(connected, bool):
                return connected
            socket_obj = getattr(client, "socket", None)
            return socket_obj is not None

    def describe(self) -> str:
        return f"modbus-tcp://{self.host}:{self.port} (unit {self.unit_id})"

    # --------------------------------------------------------------- access

    def _unit_kwargs(self) -> dict:
        return {self._unit_kw: self.unit_id} if self._unit_kw else {}

    def read_holding(self, address: int, count: int) -> List[int]:
        with self._lock:
            if self._client is None:
                raise ModbusError("Not connected")
            try:
                if self._count_is_kw:
                    result = self._client.read_holding_registers(
                        address, count=count, **self._unit_kwargs()
                    )
                else:
                    result = self._client.read_holding_registers(
                        address, count, **self._unit_kwargs()
                    )
            except Exception as exc:
                raise ModbusError(
                    f"Read of {count} word(s) at {address} failed: {exc}"
                ) from exc
            if result is None or (hasattr(result, "isError") and result.isError()):
                raise ModbusError(
                    f"Read of {count} word(s) at {address} returned an error: {result}"
                )
            registers = list(getattr(result, "registers", []) or [])
            if len(registers) != count:
                raise ModbusError(
                    f"Read at {address} expected {count} word(s), got {len(registers)}"
                )
            return registers

    def write_holding(self, address: int, values: List[int]) -> None:
        with self._lock:
            if self._client is None:
                raise ModbusError("Not connected")
            payload = [int(v) & 0xFFFF for v in values]
            try:
                result = self._client.write_registers(
                    address, payload, **self._unit_kwargs()
                )
            except Exception as exc:
                raise ModbusError(
                    f"Write of {payload} at {address} failed: {exc}"
                ) from exc
            if result is None or (hasattr(result, "isError") and result.isError()):
                raise ModbusError(
                    f"Write of {payload} at {address} returned an error: {result}"
                )


__all__ = ["Transport", "PymodbusTransport", "ModbusError"]
