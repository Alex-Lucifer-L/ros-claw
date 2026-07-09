def main():
    routes={
        "/api": "default_service",
        "/api/robot": "robot_service", 
        "/api/vision": "vision_service",
        "/api/payment": "payment_service",
    }

    ###这是注册表

    services={
        "default_service": {"url": "http://localhost:8000","status":"healthy"},
        "robot_service": {"url": "http://localhost:8001","status":"healthy"},
        "vision_service": {"url": "http://localhost:8002","status":"healthy"},
    }
    ###这是服务注册表

    route=input("Enter the route: ")
    ###这是路由输入
    token=input("Enter the token: ")


    permissions={
        "student_token": ["robot_service"],
        "vision_token": ["vision_service"],
        "admin_token": ["default_service", "robot_service", "vision_service"]
    }
    logs=[]
    result=handle_request(route, token, routes, services, permissions)
    logs.append(result)
    print(logs)
    
def handle_request(route, token, routes, services, permissions):
    result={
        "route": route,
        "token": token,
    }
    route_service = route_to_service(routes, route)
    if not route_service:
        add_log(result, "service_name", None)
        add_log(result, "allow", False)
        add_log(result, "message", "DENY: No route found")
        return result

    if not check_token(token, permissions):
        add_log(result, "service_name", route_service)
        add_log(result, "allow", False)
        add_log(result, "message", "DENY: Invalid token")
        return result

    if not check_permission(token, route_service, permissions):
        add_log(result, "service_name", route_service)
        add_log(result, "allow", False)
        add_log(result, "message", "DENY: Permission denied")
        return result

    services_info = check_service_info(services, route_service)
    if not services_info:
        add_log(result, "service_name", route_service)
        add_log(result, "allow", False)
        add_log(result, "message", "DENY: Service not registered")
        return result

    if check_service_health(services_info):
        add_log(result, "service_name", route_service)
        add_log(result, "allow", True)
        add_log(result, "message", f"ALLOW: routing to service {route_service} at {services_info['url']}")
        return result

    add_log(result, "service_name", route_service)
    add_log(result, "allow", False)
    add_log(result, "message", "DENY: Service is unhealthy")
    return result
    ###这是请求处理函数

def add_log(log,key,value):
    log[key]=value
    return log
    ###这是日志添加函数

def route_to_service(routes, path):
    matches_key=[]
    for key in routes:
        if path.startswith(key):
            matches_key.append(key)
    longest_match = max(matches_key, key=len) if matches_key else None
    return routes[longest_match] if longest_match else None
    ###这是路由最大匹配函数


def check_service_info(services, service_name):
    if service_name in services:
        service_info = services[service_name]
        return service_info
    else:
        return None
    ###这是服务注册表查询函数

def check_service_health(service_info):
    if service_info["status"] == "healthy":
        return True
    else:
        return False
    ###这是服务健康检查函数

def check_token(token, permissions):
    if token in permissions:
        return True
    else:
        return False
    ###这是token检查函数
def check_permission(token, service_name,permissions):
    if token in permissions:
        if service_name in permissions[token]:
            return True
    return False
    ###这是权限检查函数


if __name__ == "__main__":
    main()  
