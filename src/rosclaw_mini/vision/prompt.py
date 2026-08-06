"""构建只读场景观察 Prompt。"""


DEFAULT_VISION_QUESTION = "描述桌面上有什么物体以及它们在画面中的大致位置。"


def build_vision_prompt(question: str | None = None) -> str:
    user_question = (
        question.strip()
        if isinstance(question, str) and question.strip()
        else DEFAULT_VISION_QUESTION
    )
    return f"""你是 rosclaw-mini 的只读场景观察器。
请观察当前单张图像，描述所有实际可见内容，并回答用户问题。

强制边界：
1. 只能描述图像中实际可见内容；不确定时写 unknown，禁止虚构。
2. 不得因为图像来自腕部摄像头，或提示词提到机械臂，就把普通剪刀、
   夹子、工具手柄或其他孤立物体猜测为机械臂夹爪。
3. 只有清晰看到物体与机械连杆、执行器或其他机械结构相连时，
   才能命名为“机械臂夹爪”。只看到手柄状、弯曲或被遮挡的孤立物体时，
   name/category 应使用中性名称或 unknown，降低 confidence，并在 warnings 中说明候选类别。
4. 禁止输出机械臂基座坐标、TCP 三维坐标或任何 x/y/z 世界坐标。
5. 禁止输出 Command、Skill、运动指令、抓取计划或控制建议。
6. location_in_image 只能是 left、center、right、upper_left、
   upper_center、upper_right、lower_left、lower_center、lower_right、unknown。
7. confidence 无法可靠判断时为 null，否则必须在 0 到 1。
8. bounding_box 无法可靠判断时为 null；存在时为图像归一化语义估计
   [x_min,y_min,x_max,y_max]，每项在 0 到 1。它不是精密检测结果。
9. warnings 记录遮挡、模糊、过曝、欠曝、光照不足、类别不确定或画面外截断。

只返回一个 JSON 对象，不要 Markdown、代码块或解释。JSON 结构严格为：
{{
  "scene_description": "场景概述",
  "objects": [
    {{
      "name": "物体名称",
      "category": null,
      "color": null,
      "location_in_image": "unknown",
      "confidence": null,
      "attributes": {{}},
      "bounding_box": null
    }}
  ],
  "warnings": []
}}

用户视觉问题：{user_question}
"""


def build_localization_prompt(
    question: str,
    *,
    image_width: int,
    image_height: int,
) -> str:
    """Build the one-target prompt using model-native integer pixel boxes.

    Qwen's grounding output is materially more accurate when it reports source
    image pixels.  The application, rather than the model, normalizes those
    integers before exposing the regular ``SceneObservation`` schema.
    """

    if not isinstance(question, str) or not question.strip():
        raise ValueError("目标定位问题不能为空。")
    for name, value in (("image_width", image_width), ("image_height", image_height)):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"{name} 必须是正整数。")
    return f"""你是 rosclaw-mini 的只读目标选择和视觉 grounding 器。
当前原始 RGB 图像尺寸是 {image_width}×{image_height} 像素。
请观察这一张图像，并回答用户的目标定位问题。

强制边界：
1. 只能选择一个含义明确、真实可见的目标；不得猜测。
2. 存在多个同样匹配的候选、目标不可见、被严重遮挡，
   或无法给出可靠边界时，objects 必须返回空列表并在 warnings 说明。
3. objects 非空时只能有一项。bounding_box_pixels 必须是
   [x_min,y_min,x_max,y_max] 四个整数像素坐标；左上为 (0,0)，
   右下边界为 ({image_width},{image_height})。
4. 边界框必须紧致包含目标的完整可见外轮廓；不得包含大片背景，
   不得裁掉目标下半部或把桌面当作目标。
5. 不得输出 0 到 1 的小数 bounding_box；bounding_box 字段固定为 null。
   程序会在严格检查像素坐标后自行归一化。
6. 禁止输出机械臂基座坐标、TCP 三维坐标、Command、Skill、
   运动指令、抓取计划或控制建议。
7. location_in_image 只能是 left、center、right、upper_left、
   upper_center、upper_right、lower_left、lower_center、lower_right、unknown。
8. confidence 无法可靠判断时为 null，否则必须在 0 到 1。

只返回一个 JSON 对象，不要 Markdown、代码块或解释。JSON 结构严格为：
{{
  "scene_description": "场景概述",
  "objects": [
    {{
      "name": "目标名称",
      "category": null,
      "color": null,
      "location_in_image": "unknown",
      "confidence": null,
      "attributes": {{}},
      "bounding_box": null,
      "bounding_box_pixels": [0, 0, {image_width}, {image_height}]
    }}
  ],
  "warnings": []
}}

用户目标定位问题：{question.strip()}
"""
