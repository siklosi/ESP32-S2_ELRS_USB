Import("env")
import os

def before_upload(source, target, env):
    print("Preparing SPIFFS upload")
    data_dir = os.path.join(env.get("PROJECT_DIR"), "data")
    env.Execute(f"pio run -t buildfs")
    env.Execute(f"pio run -t uploadfs")

env.AddPreAction("upload", before_upload)