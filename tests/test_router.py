from src.rosclaw_mini.command_schema.schemas import GatewayRequest, Route
from src.rosclaw_mini.gateway.router import match_route


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

matched_route = match_route(request, routes)

print(matched_route)