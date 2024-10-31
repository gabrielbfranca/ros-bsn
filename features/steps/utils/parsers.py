import subprocess
def capture_csv_data(topic, message_keys, line_limit=10):
    """
    Capture CSV data from a ROS2 topic and organize it into a dictionary with keys based on message keys.
    """
    process = subprocess.Popen(
        ['ros2', 'topic', 'echo', '--csv', topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    # Initialize the output dictionary with empty lists for each key
    output = {key: [] for key in message_keys.split(',')}
    try:
        # Read lines until reaching the line limit
        for line in process.stdout:
            values = line.strip().split(',')  # Split the CSV line into values
            if len(values) == len(output):  # Ensure the line has the correct number of values
                # Map values to their respective keys in the output dictionary
                for key, value in zip(output.keys(), values):
                    # Convert value to bool if 'True' or 'False', otherwise convert to int
                    output[key].append(value)

            if sum(len(v) for v in output.values()) >= line_limit:
                process.terminate()  # Terminate the subprocess once limit is reached
                break
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        process.wait()  # Ensure the process exits properly

    return output

def get_rosnode_info_ros2(lines):
    node_info = {
        "publishers": [],
        "subscribers": [],
        "services": {
            "servers": [],
            "clients": []
        },
        "actions": {
            "servers": [],
            "clients": []
        }
    }

    def parse_topic_lines(start_index, keyword):
        topics = []
        i = start_index
        while i < len(lines) and lines[i].strip().startswith(keyword):
            line = lines[i].strip().split(': ')
            if len(line) == 2:
                topic = line[0].strip()
                msg_type = line[1].strip()
                topics.append({"topic": topic, "type": msg_type})
            i += 1
        return topics, i

    i = 0
    while i < len(lines):
        line = lines[i].strip()

        if line.startswith('Subscribers:'):
            node_info["subscribers"], i = parse_topic_lines(i + 1, '/')

        elif line.startswith('Publishers:'):
            node_info["publishers"], i = parse_topic_lines(i + 1, '/')

        elif line.startswith('Service Servers:'):
            i += 1
            while i < len(lines) and lines[i].strip().startswith('/'):
                service = lines[i].strip().split(': ')[0]
                node_info["services"]["servers"].append(service)
                i += 1

        elif line.startswith('Service Clients:'):
            i += 1
            while i < len(lines) and lines[i].strip().startswith('/'):
                client = lines[i].strip().split(': ')[0]
                node_info["services"]["clients"].append(client)
                i += 1

        elif line.startswith('Action Servers:'):
            i += 1
            while i < len(lines) and lines[i].strip().startswith('/'):
                action_server = lines[i].strip().split(': ')[0]
                node_info["actions"]["servers"].append(action_server)
                i += 1

        elif line.startswith('Action Clients:'):
            i += 1
            while i < len(lines) and lines[i].strip().startswith('/'):
                action_client = lines[i].strip().split(': ')[0]
                node_info["actions"]["clients"].append(action_client)
                i += 1
        else:
            i += 1

    return node_info
