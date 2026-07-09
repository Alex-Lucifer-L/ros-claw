from ..command_schema.schemas import GatewayRequest, Route

def match_route(request: GatewayRequest, routes: list[Route]) -> Route | None:
    """
    匹配请求路径与路由列表中的路径前缀，返回匹配的路由对象。
    如果没有匹配的路由，返回None。
    """
    matched_routes = []
    for route in routes:
        if request.path.startswith(route.path_prefix):
            matched_routes.append(route)
    max_matched_route = max(matched_routes, key=lambda route: len(route.path_prefix), default=None)
    return max_matched_route