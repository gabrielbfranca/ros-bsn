from behave import given, when, then
import subprocess
import concurrent.futures
from utils.parsers import capture_csv_data

@given('the {topic_name} topic is online')
def step_given_topic_is_online(context, topic_name):
    result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    topic_list = result.stdout.decode('utf-8').splitlines()
    assert topic_name in topic_list, f"{topic_name} is not online"

@when('I listen to topics')
def step_when_listen_to_topics(context):
    context.topic_Data = {}

    # Use ThreadPoolExecutor to capture data from each topic in parallel
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = {}
        for row in context.table:
            topic = row['Topic Name']
            message_keys = row['Data Type']  # Keys from messages, e.g., 'num,data'
            line_limit = int(row.get('Line Limit', 10))  # Optional line limit per topic

            # Submit the capture task to the executor
            futures[executor.submit(capture_csv_data, topic, message_keys, line_limit)] = topic

        # Retrieve the captured data once all tasks are completed
        for future in concurrent.futures.as_completed(futures):
            topic = futures[future]
            try:
                context.topic_Data[topic] = future.result()
            except Exception as e:
                print(f"Error while capturing data from {topic}: {e}")

@then('registration will process data')
def step_then_process_risks(context):
    assert context.topic_Data, "No data collected from sensor topics."
    
