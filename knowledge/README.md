# rosclaw-mini 项目知识索引

本目录是 LLM 命令生成前使用的静态项目知识，不是机械臂实时状态库。
`RagContextProvider` 会稳定排序加载除本索引外的 Markdown，按二级标题
切块，并用 `KeywordRetriever` 只取少量相关块。

| 文档 | 职责 | 主要事实来源 |
| --- | --- | --- |
| `project_overview.md` | 总体链路和模块边界 | `main.py`、`runtime.py`、Gateway、Controller |
| `command_and_skill_contracts.md` | Command/Result/Skill/参数契约 | command schema、`arm_skills.py`、测试 |
| `safety_rules.md` | Validator、Safety、坐标和工作空间 | safety、session、trajectory validation |
| `session_states.md` | REST/TRANSITION/WORK/UNVERIFIED | `so100_plus_session.py`、状态测试 |
| `arm_backends.md` | Mock、真机、Adapter/Handler | arm、factory、runtime |
| `execution_and_stop.md` | 线程、Lock、Event、stop、shutdown | controller、runtime、stop 测试 |
| `llm_command_interface.md` | 千问/OpenAI-compatible 协议与 Prompt | llm 包、main、LLM 测试 |
| `gateway_and_permissions.md` | Gateway 顺序和未接入脚手架 | Gateway、Registry、schemas |
| `testing_and_operations.md` | 无硬件运行、错误定位和限制 | pyproject、main、runtime、测试 |

## 静态知识与实时状态

静态知识包括代码中定义的协议、Skill 名称、状态规则、坐标约定和认证配置
的职责。当前 TCP、关节角、夹爪位置、会话状态、温度、电流、负载和现场
障碍物必须由运行时/硬件反馈提供。不得把文档中的例子或历史验收值当成
当前观测。

## 更新原则

事实优先级是可执行代码和配置，其次测试、正式文档、注释与示例。修改
Skill、状态机、安全边界、入口或错误语义时，应更新对应主题的摘要、
`source_files` 和版本。不要大段复制源码；记录稳定的行为、责任边界及
可追溯相对路径。新增可检索文档必须提供唯一 `document_id` 和 Loader
要求的 Front Matter；仅作为索引的文件名应保持 `README.md`。

第一版关键词检索不理解深层语义或同义词。未来可以实现相同 `Retriever`
接口的 Embedding/VectorRetriever；替换检索器不应改变 Prompt 的来源边界，
更不能改变 Parser、Validator、Gateway、Safety Checker 或会话门禁。
