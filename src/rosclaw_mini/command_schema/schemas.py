from dataclasses import dataclass

@dataclass
class GatewayRequest:
    """
    网关请求的结构体:
    这个结构体里包含了网关请求的所有必要信息:
    请求ID： 用于唯一标识每个请求，方便追踪和调试。
    路径： 请求的目标路径，用于路由到相应的服务。
    方法： HTTP请求方法，如GET、POST、PUT、DELETE等。
    
    """
    request_id: str
    path: str
    method: str
   

@dataclass
class Route:
    """
    路由的结构体：
    这个结构体定义了网关路由的基本信息，包括路径前缀、服务名称和权限要求
    路径前缀： 用于匹配请求的路径，决定请求应该路由到哪个服务。
    服务名称： 指定请求应该路由到的目标服务的名称。
    权限要求： 定义访问该路由所需的权限级别，可能的值为：'public'、'authenticated'、'admin'等，用于控制访问

    """
    path_prefix: str
    service_name: str
    permission_required: str
    

@dataclass
class Service:
    """
    服务的结构体：
    这个结构体定义了网关服务的基本信息，包括服务名称、URL、状态和启用状态
    服务名称： 用于唯一标识服务，通常是一个简短的字符串。
    URL： 服务的访问地址，通常是一个完整的URL，用于路由请求
    状态： 服务的当前状态，可能的值为：'running'、'stopped'、'error'等，用于监控和管理服务。
    启用状态： 一个布尔值，指示服务是否启用。如果为True，网关将允许请求路由到该服务；如果为False，网关将拒绝请求路由到该服务。
    """
    name: str
    url: str
    status: str
    enabled: bool

@dataclass
class Permission:
    """
    权限的结构体：
    这个结构体定义了网关权限的基本信息，包括权限名称、风险级别和描述
    权限名称： 用于唯一标识权限，通常是一个简短的字符串。
    风险级别： 定义权限的风险级别，可能的值为： 'low'、'medium'、'high'等，用于评估权限的安全性
    描述： 对权限的详细描述，解释权限的用途和访问范围
    """
    name: str
    risk_level: str
    description: str

@dataclass
class GatewayResponse:
    """
    网关响应的结构体
    这个结构体定义了网关响应的基本信息，包括请求是否成功、请求ID、状态码、消息、数据、目标服务、目标URL和错误信息
    是否成功： 一个布尔值，指示请求是否成功处理。
    请求ID： 用于唯一标识每个请求，方便追踪和调试。
    状态码： HTTP状态码，表示请求的处理结果。
    消息： 对请求处理结果的简短描述，通常用于提示用户或开发者。

    """
    success: bool
    request_id: str
    status_code: int
    message: str
