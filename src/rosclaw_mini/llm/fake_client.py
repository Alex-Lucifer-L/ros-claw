class FakeLLMClient:

    def __init__(self,response: str)-> None:
        """
        初始化FakeLLMClient实例。该方法用于创建一个模拟的LLM客户端对象。
        """
        self.response = response

    def generate(self, prompt: str) -> str:
        """
        模拟调用LLM，生成统一的JSON格式的指令。该方法接受一个字符串类型的 prompt 参数，表示输入的提示信息，并返回一个字符串类型的结果，表示生成的JSON格式指令。
        """
        # 这里可以根据需要返回一个固定的JSON格式指令，或者根据 prompt 生成不同的指令
        return self.response