# simple_astar_planner/prompt_branch/plan_prompt.py
from langchain_core.prompts import ChatPromptTemplate, FewShotChatMessagePromptTemplate
from langchain_openai import ChatOpenAI


examples = [
    {
        "input": "책상 앞에 가서 사과가 있으면 사진 찍고, 없으면 대기해.",
        "output": """{
            "plan": {
                "setup": [
                    {"action":"goto","x":-3.0,"y":-7.0,"yaw":3.14,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"detection","x":null,"y":null,"yaw":null,"duration":null,"class":"apple","timeout":5.0,"arg":null,"target":null}
                ],
                "branches": {
                    "if_true": [
                        {"action":"photo","x":null,"y":null,"yaw":null,"duration":null,"class":null,"timeout":null,"arg":null,"target":null}
                    ],
                    "if_false": [
                        {"action":"wait","x":null,"y":null,"yaw":null,"duration":3.0,"class":null,"timeout":null,"arg":null,"target":null}
                    ]
                }
            }
        }"""
    },
        {
        "input": "책상 앞에 가서 스포츠볼 찾아봐. 있으면 사진 찍고 없으면 대기해.",
        "output": """{
            "plan": {
                "setup": [
                    {"action":"goto","x":-3.0,"y":-7.0,"yaw":3.14,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"detection","x":null,"y":null,"yaw":null,"duration":null,"class":"sports ball","timeout":5.0,"arg":null,"target":null}
                ],
                "branches": {
                    "if_true": [
                        {"action":"photo","x":null,"y":null,"yaw":null,"duration":null,"class":null,"timeout":null,"arg":null,"target":null}
                    ],
                    "if_false": [
                        {"action":"wait","x":null,"y":null,"yaw":null,"duration":3.0,"class":null,"timeout":null,"arg":null,"target":null}
                    ]
                }
            }
        }"""
    },
        {
        "input": "소파 앞에 가서 스포츠볼이 찾아봐, 있으면 사진 찍고 없으면 대기해.",
        "output": """{
            "plan": {
                "setup": [
                    {"action":"goto","x":-5.0,"y":-7.0,"yaw":0,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"detection","x":null,"y":null,"yaw":null,"duration":null,"class":"sports ball","timeout":5.0,"arg":null,"target":null}
                ],
                "branches": {
                    "if_true": [
                        {"action":"photo","x":null,"y":null,"yaw":null,"duration":null,"class":null,"timeout":null,"arg":null,"target":null}
                    ],
                    "if_false": [
                        {"action":"wait","x":null,"y":null,"yaw":null,"duration":3.0,"class":null,"timeout":null,"arg":null,"target":null}
                    ]
                }
            }
        }"""
    },
    {
        "input": "소파 앞에 가서 사진 찍고 책상 앞으로 가.",
        "output": """{
            "plan": {
                "setup": [
                    {"action":"goto","x":-5.0,"y":-7.0,"yaw":0,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"photo","x":null,"y":null,"yaw":null,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"goto","x":-3.0,"y":-7.0,"yaw":0,"duration":null,"class":null,"timeout":null,"arg":null,"target":null}
                ],
                "branches": {
                    "if_true": [],
                    "if_false": []
                }
            }
        }"""
    },
        {
        "input": "책상 한 바퀴 돌아",
        "output": """{
            "plan": {
                "setup": [
                    {"action":"goto","x":-5.0,"y":-7.0,"yaw":0,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"goto","x":-2.5,"y":-9.0,"yaw":0,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"goto","x":0.0,"y":-7.0,"yaw":0,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"goto","x":-2.5,"y":-5.0,"yaw":0,"duration":null,"class":null,"timeout":null,"arg":null,"target":null},
                    {"action":"goto","x":-5.0,"y":-7.0,"yaw":0,"duration":null,"class":null,"timeout":null,"arg":null,"target":null}
                ],
                "branches": {
                    "if_true": [],
                    "if_false": []
                }
            }
        }"""
    },
    {
    "input": "책상 주위를 돌면서 스포츠볼 찾아봐.",
    "output": """{
        "plan": {
            "setup": [
                {"action": "goto", "x": -5.0, "y": -7.0, "yaw": 0.0, "duration": null, "class": null, "timeout": null, "arg": null, "target": "desk.start"},
                {"action": "detection", "x": null, "y": null, "yaw": null, "duration": null, "class": "sports ball", "timeout": 5.0, "arg": null, "target": null}
            ],
            "branches": {
                "if_true": [
                    {"action": "photo", "x": null, "y": null, "yaw": null, "duration": null, "class": null, "timeout": null, "arg": null, "target": null}
                ],
                "if_false": [
                    {"action": "goto", "x": -3.5, "y": -9.0, "yaw": 1.57, "duration": null, "class": null, "timeout": null, "arg": null, "target": "desk.south"},
                    {"action": "detection", "x": null, "y": null, "yaw": null, "duration": null, "class": "sports ball", "timeout": 5.0, "arg": null, "target": null},
                    {
                        "branches": {
                            "if_true": [
                                {"action": "photo", "x": null, "y": null, "yaw": null, "duration": null, "class": null, "timeout": null, "arg": null, "target": null}
                            ],
                            "if_false": [
                                {"action": "goto", "x": -2.0, "y": -7.0, "yaw": 3.14, "duration": null, "class": null, "timeout": null, "arg": null, "target": "desk.right"},
                                {"action": "detection", "x": null, "y": null, "yaw": null, "duration": null, "class": "sports ball", "timeout": 5.0, "arg": null, "target": null},
                                {
                                    "branches": {
                                        "if_true": [
                                            {"action": "photo", "x": null, "y": null, "yaw": null, "duration": null, "class": null, "timeout": null, "arg": null, "target": null}
                                        ],
                                        "if_false": [
                                            {"action": "goto", "x": -3.5, "y": -5.0, "yaw": -1.57, "duration": null, "class": null, "timeout": null, "arg": null, "target": "desk.north"},
                                            {"action": "detection", "x": null, "y": null, "yaw": null, "duration": null, "class": "sports ball", "timeout": 5.0, "arg": null, "target": null},
                                            {
                                                "branches": {
                                                    "if_true": [
                                                        {"action": "photo", "x": null, "y": null, "yaw": null, "duration": null, "class": null, "timeout": null, "arg": null, "target": null}
                                                    ],
                                                    "if_false": [
                                                        {"action": "goto", "x": -5.0, "y": -7.0, "yaw": 0.0, "duration": null, "class": null, "timeout": null, "arg": null, "target": "desk.start"},
                                                        {"action": "wait", "x": null, "y": null, "yaw": null, "duration": 2.0, "class": null, "timeout": null, "arg": null, "target": null}
                                                    ]
                                                }
                                            }
                                        ]
                                    }
                                }
                            ]
                        }
                    }
                ]
            }
        }
    }"""
}

    
]


# ───────────────────────────────────────────────
# ② Few-shot 템플릿 구성
# ───────────────────────────────────────────────
example_prompt = ChatPromptTemplate.from_messages([
    ("human", "{input}"),
    ("ai", "{output}"),
])

few_shot_prompt = FewShotChatMessagePromptTemplate(
    example_prompt=example_prompt,
    examples=examples,
)

SYSTEM_PROMPT = (
    "너는 ROS2 기반 이동 로봇의 시퀀스 플래너다. "
    "오직 JSON만 출력하고, 항상 plan.setup[], plan.branches.if_true[], plan.branches.if_false[] 구조를 따라야 한다. "
    "모든 step은 action, x, y, yaw, duration, class, timeout, arg, target 키를 가져야 한다. "
    "각 action 규칙은 다음과 같다:\n"
    "- goto: class는 항상 null이며, 이동 좌표(x,y)는 anchors의 target 이름을 기준으로 결정한다.\n"
    "- detection: class는 탐지할 객체명이며, timeout은 5초 기본값을 가진다.\n"
    "- photo: class와 target은 null.\n"
    "- wait: duration만 유효하며 class는 null.\n"
    "모든 좌표는 반드시 주어진 anchors 목록에서 target 이름을 찾아서 설정해야 한다. "
    "forbidden_bboxes 영역 내 좌표는 절대 사용 금지다. "
    "답변은 JSON 이외의 텍스트를 포함하지 않는다. "
    "사용하지 않는 field는 null로 채운다. "
    "단, branches 내부에도 setup과 branches를 포함한 하위 plan 구조를 중첩할 수 있다 (nested branching 허용). "
    "모든 JSON은 완전한 문법 형식을 따라야 하며 쉼표(,) 누락이나 닫는 괄호 누락 없이 유효한 JSON으로 생성한다. "
    "branch의 중첩은 최대 2단계까지만 허용한다."
)



final_prompt = ChatPromptTemplate.from_messages([
    ("system", SYSTEM_PROMPT),
    few_shot_prompt,
    ("human", "{user_command}")
])


# ───────────────────────────────────────────────
# ④ 체인 생성 함수
# ───────────────────────────────────────────────
def make_plan_chain():
    model = ChatOpenAI(model="gpt-4o-mini", temperature=0.0).bind(
        response_format={"type": "json_object"}  # ✅ JSON 모드
    )
    chain = final_prompt | model
    return chain

