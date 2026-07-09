from src.rosclaw_mini.command_schema.schemas import Service
from src.rosclaw_mini.gateway.registry import find_service_by_name

services = [
    Service(
        name="default_service",
        url="http://localhost:8000",
        status="healthy",
        enabled=True
    ),
    Service(
        name="robot_service",
        url="http://localhost:8001",
        status="healthy",
        enabled=True
    ),
    Service(
        name="vision_service",
        url="http://localhost:8002",
        status="healthy",
        enabled=True
    ),
]

service = find_service_by_name(services, "robot_service")

print(service)