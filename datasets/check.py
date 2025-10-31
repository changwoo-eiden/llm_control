from openai import OpenAI
client = OpenAI()  # OPENAI_API_KEY는 환경변수로

JOB_ID = "ftjob-FSiwwR0R0y2NpB2VyXR04S17"  # 너의 job_id
j = client.fine_tuning.jobs.retrieve(JOB_ID)
print("status =", j.status)                 # succeeded / running / queued / failed ...
print("fine_tuned_model =", j.fine_tuned_model)  # 완료되면 모델 ID가 표시됨



ev = client.fine_tuning.jobs.list_events("ftjob-FSiwwR0R0y2NpB2VyXR04S17", limit=50)
for e in ev.data:
    print(e.level, e.message)

import json

path = "/home/changwoo/ros2_ws/datasets/desk_dataset.jsonl"  # 실제 경로로 변경

with open(path, "r", encoding="utf-8") as f:
    for i, line in enumerate(f, start=1):
        s = line.strip()
        if not s:
            print(f"[빈줄] 라인 {i}")
            continue
        try:
            obj = json.loads(s)
        except json.JSONDecodeError as e:
            print(f"[형식오류] 라인 {i}: {e}")
            print(s[:200])
            break
        if not isinstance(obj, dict):
            print(f"[오류] 라인 {i}: 최상위가 객체가 아님")
            break
        msgs = obj.get("messages")
        if not isinstance(msgs, list):
            print(f"[오류] 라인 {i}: 'messages'가 리스트 아님")
            break
        for m in msgs:
            if not (isinstance(m, dict) and "role" in m and "content" in m):
                print(f"[오류] 라인 {i}: message 형식 잘못됨 -> {m}")
                break
