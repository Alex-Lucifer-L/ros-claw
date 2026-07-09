from ..command_schema.schemas import Service

def find_service_by_name(services:list[Service], service_name:str) -> Service | None:
    """
    根据服务名称在服务列表中查找对应的服务对象。
    如果找到，返回服务对象；如果未找到，返回None。
    """
    for service in services:
        if service.name == service_name:
            return service
    return None