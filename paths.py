from pathlib import Path

PROJECT_PATH = Path(__file__).resolve().parent

WEB_APP_PATH = PROJECT_PATH / 'web_app'

SCRIPTS_PATH = PROJECT_PATH / 'scripts'

TRAJECTORIES_PATH = PROJECT_PATH / 'SampleTraj'

print(PROJECT_PATH)
print(WEB_APP_PATH)
print(SCRIPTS_PATH)
print(TRAJECTORIES_PATH)