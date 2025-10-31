from openai import OpenAI
import os


client = OpenAI(api_key=
                "REMOVED_KEYproj-H6K2SPF92wcyjsbAy_V2JSmE_uy1jidEOTn3F1afyTwQPNTeEkjT2IBmYzVL81XwGk1WGqxvjXT3BlbkFJRzMNN5ZcU1tQ4WcURKpguRavh7b3ZcaEcqs5iCZpWK3eoebIh25jH3NgTqNVPjssuosXEmVBwA")

upload = client.files.create(
    file=open("/home/changwoo/ros2_ws/datasets/desk_dataset.jsonl", "rb"),
    purpose="fine-tune"
)
print("training_file_id =", "file-C3tLrWcJPxmeKbLARgU8vG")

job = client.fine_tuning.jobs.create(
    model="gpt-4o-mini-2024-07-18",            
    training_file= "file-C3tLrWcJPxmeKbLARgU8vG"
)

print("job_id =", job.id)
print("status =", job.status) 

j = client.fine_tuning.jobs.retrieve(job.id)
print(j.status)                 # validating_files / queued / running / succeeded / failed
print(j.fine_tuned_model)       # succeeded가 되면 값이 생김
