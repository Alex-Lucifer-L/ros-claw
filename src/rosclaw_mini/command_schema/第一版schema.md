##主要数据名词有
    GatewayRequest
    Route
    Service
    Permission
    GatewayResponse


    GatewayRequest:
    - request_id
    - path
    - method
    - payload
    - source

    Route:
    - path_prefix
    - service_name
    - permission_required

    Service:
    - name
    - url
    - status
    - enabled

    Permission:
    - name
    - risk_level
    - description

    GatewayResponse:
    - success
    - status_code
    - message
    - target_service
    - target_url
    - error