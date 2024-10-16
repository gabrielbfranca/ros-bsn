
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
