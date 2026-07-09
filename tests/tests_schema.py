from src.rosclaw_mini.command_schema.schemas import GatewayRequest, Route, Service, Permission, GatewayResponse
request = GatewayRequest(
    request_id="req-001",
    path="/api/robot",
    method="GET"
)

route = Route(
    path_prefix="/api/robot",
    service_name="robot_service",
    permission_required="public"
)

service = Service(
    name="robot_service",
    url="http://localhost:8001",
    status="healthy",
    enabled=True
)

permission = Permission(
    name="public",
    risk_level="low",
    description="公开访问权限"
)

response = GatewayResponse(
    success=True,
    request_id="req-001",
    status_code=200,
    message="请求成功匹配到 robot_service"
)

print(request)
print(route)
print(service)
print(permission)
print(response)