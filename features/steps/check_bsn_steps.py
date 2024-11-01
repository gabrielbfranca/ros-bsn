from behave import given, when, then
import subprocess
import concurrent.futures
from utils.parsers import capture_csv_data
def count_matching_elements(list1, list2):
    matching_elements = set(list1) & set(list2)
    # Return the count of matching elements
    return len(matching_elements)

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

@then('sensors will process the risks')
def step_then_check_high_risk(context):
    assert any(context.sensor_data.values()), "No risk data found in sensor topics."

    for topic, data in context.sensor_data.items():
        assert 'risk' in data and data['risk'], f"No risk data detected in topic {topic}."

@then('/TargetSystemData will receive the risks from sensors')
def step_then_check_target_system_receives_risk(context):
    #print(f'TargetSystemData is receiving the risk data from sensors: {context.target_system_data}')
    risk_key_mapping = {
    '/thermometer_data': 'trm_risk',
    '/ecg_data': 'ecg_risk',
    '/oximeter_data': 'oxi_risk',
    '/abps_data': 'abps_risk',
    '/abpd_data': 'abpd_risk',
    '/glucosemeter_data': 'glc_risk',
    }
    print("TARGET SYSTEM DATA: ", context.target_system_data)
    target_system_data = context.target_system_data
    print(f'TARGET sytem data risks: {target_system_data}')
    sensor_data = context.sensor_data
    print(f'SENSOR DATA: {sensor_data}')
    for key, value in risk_key_mapping.items():
    
        print("Target:", key, "Sensor:", value)
        print(f"Target risks: {sensor_data[key]['risk']} Sensor risks: {target_system_data[value]}")
        elements = count_matching_elements(sensor_data[key]['risk'], target_system_data[value])
        assert elements > 0, f"Topics {key} and {value} do not have matching risk data."
@then('sensors will process the data')
def step_check_if_sensors_process_data(context):
    assert any(context.sensor_data.values()), "No risk data found in sensor topics."

    for topic, data in context.sensor_data.items():
        assert 'data' in data and data['data'], f"No risk data detected in topic {topic}."
        
@then('/TargetSystemData will receive the data from sensors')
def step_check_if_TargetSystem_process_data(context):
    data_key_mapping = {
    '/thermometer_data': 'trm_data',
    '/ecg_data': 'ecg_data',
    '/oximeter_data': 'oxi_data',
    '/abps_data': 'abps_data',
    '/abpd_data': 'abpd_data',
    '/glucosemeter_data': 'glc_data',
    }
    print(f'TARGET sytem data: {context.target_system_data}')
    target_data= context.target_system_data
    sensor_data = context.sensor_data
    print(f'TARGET sytem data risks: {target_data}')
    print(f'SENSOR DATA: {sensor_data}')
    for key, value in data_key_mapping.items():
    
        print("Target:", key, "Sensor:", value)
        print(f"Target risks: {sensor_data[key]['data']} Sensor data: {target_data[value]}")
        elements = count_matching_elements(sensor_data[key]['data'], target_data[value])
        assert elements > 0, f"Topics {key} and {value} do not have matching risk data."