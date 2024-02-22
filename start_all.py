import subprocess

CONDITION = 'GOA'
MAP_NUMBER = 't2'

settings_file_path = './scenarios/settings_{}.yaml'.format(MAP_NUMBER)
temp_settings_path = './data/settings_{}.yaml'.format(MAP_NUMBER)

with open(settings_file_path, 'r') as f:
    settings = f.read()
    settings = settings.replace('CONDITION', CONDITION)
with open(temp_settings_path, 'w') as f:
    f.write(settings)

p = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', './start_ui.sh {}'.format(temp_settings_path)], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
p = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', './start_waypoint.sh {}'.format(temp_settings_path)], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
p = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', './start_concurrent.sh {}'.format(temp_settings_path)], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
p = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', './start_bridge.sh'], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
