from enum import Enum
from typing import List
from uuid import UUID, uuid4
from pydantic import BaseModel, Field

class ZoneType(str, Enum):
    ROOM = "room"
    NO_GO = "no_go"
    VIRTUAL_WALL = "virtual_wall"

class Point(BaseModel):
    x: float
    y: float

class Zone(BaseModel):
    id: UUID = Field(default_factory=uuid4)
    name: str
    type: ZoneType
    coordinates: List[Point]

class CleaningRequest(BaseModel):
    zone_ids: List[str]
    suction_power: int = Field(..., ge=0, le=100, description="Suction power percentage 0-100")

class RobotStatus(BaseModel):
    battery_level: float
    is_cleaning: bool
    current_state: str

class TeleopRequest(BaseModel):
    linear_x: float
    angular_z: float
