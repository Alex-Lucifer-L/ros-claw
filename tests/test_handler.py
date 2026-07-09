from src.rosclaw_mini.command_schema.schemas import GatewayRequest, Route, Service
from src.rosclaw_mini.gateway.handler import handle_request

request = GatewayRequest(
    request_id="req-001",
    path="/api/robot/move",
    method="GET"
)

routes = [
    Route(
        path_prefix="/api",
        service_name="default_service",
        permission_required="public"
    ),
    Route(
        path_prefix="/api/robot",
        service_name="robot_service",
        permission_required="public"
    ),
    Route(
        path_prefix="/api/vision",
        service_name="vision_service",
        permission_required="public"
    ),
]

Services=[
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

response = handle_request(request, routes, Services)
print(response)