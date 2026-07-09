from ..command_schema.schemas import GatewayRequest, Route, Service, GatewayResponse
from .router import match_route
from .registry import find_service_by_name
def handle_request(request: GatewayRequest, routes: list[Route], services: list[Service]) -> GatewayResponse:
    """
    处理网关请求，匹配路由和服务，并返回相应的响应对象。
    如果匹配到路由和服务，返回成功的响应；如果未匹配到路由或服务，返回失败的响应。  

    """
    matched_route = match_route(request, routes)
    if matched_route is None:
        return GatewayResponse(
            success=False,
            request_id=request.request_id,
            status_code=404,
            message=f"未找到匹配的路由，路径前缀: {request.path}"
        )
    

    service= find_service_by_name(services, matched_route.service_name)
    if service is None:
        return GatewayResponse(
            success=False,
            request_id=request.request_id,
            status_code=404,
            message=f"未找到匹配的服务，服务名称: {matched_route.service_name}"
        )


    if not service.enabled:
        return GatewayResponse(
            success=False,
            request_id=request.request_id,
            status_code=503,
            message=f"服务不可用，服务名称: {service.name}"
        )

    if service.status != "healthy":
        return GatewayResponse(
            success=False,
            request_id=request.request_id,
            status_code=503,
            message=f"服务状态异常，服务名称: {service.name}, 状态: {service.status}"
        )
    if service.status == "healthy":
        return GatewayResponse(
            success=True,
            request_id=request.request_id,
            status_code=200,
            message=f"请求成功匹配到 {service.name}"
        )