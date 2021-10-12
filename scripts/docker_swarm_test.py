#!/usr/bin/env python3
import argparse
import yaml
from os import system
import subprocess
PROJECT_PATH = "~"
import pathlib
import time
from sync_rosplay_cmd import SyncCtrl

def kill_docker(name):
    cmd = f"docker container kill {name}"
    system(cmd)

def launch_docker(name, config, dataset_path):
    print(f"Launching docker {name}")
    current_dir = pathlib.Path(__file__).parent.resolve()
    _id = config["dataset"][name]["id"]
    workspace = config["workspace"]
    output_path = config["output_path"] + name
    swarm_config_path = dataset_path.joinpath(config["dataset"][name]["config_path"])
    bag_path = config["dataset"][name]["bag"]
    container_name = config["container_name"]
    entry_point_path = dataset_path.joinpath(config["entry_point"])
    docker_entry_point = current_dir.joinpath("docker_entrypoint.sh")

    cmd = f"""docker run --name {name} --gpus all --rm -it \
-v {workspace}:/root/swarm_ws/ \
-v {workspace}:/home/xuhao/swarm_ws/ \
-v /home/xuhao/source/:/home/xuhao/source/ \
-v {output_path}:/root/output/ \
-v {swarm_config_path}:/root/SwarmConfig/ \
-v {dataset_path}:/root/bags/ \
-v {entry_point_path}:/root/entry_point.sh \
-v {docker_entry_point}:/root/docker_entrypoint.sh \
-v {docker_entry_point}:/root/docker_entrypoint.sh \
--env='DISPLAY' --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \
--privileged \
{container_name} /root/docker_entrypoint.sh {_id} /root/bags/{bag_path}"""
    # print(cmd)
    p_docker = subprocess.Popen(f'terminator -T {name} -x "{cmd}"', shell=True, stderr=subprocess.STDOUT)
    return p_docker

def run_swarm_docker_evaluation(config):
    pids = {}
    pids_zsh = {}
    for name in config["dataset"]:
        pids[name] = launch_docker(name, config, pathlib.Path(args.config_path).parent.resolve())

    time.sleep(1.0)
    for name in config["dataset"]:
        cmd = f'terminator -T {name} --new-tab  -x docker exec -it {name} /bin/zsh'
        print(cmd)
        pids_zsh[name] =  subprocess.Popen(cmd, shell=True, stderr=subprocess.STDOUT)
    
    return pids, pids_zsh

if __name__ == '__main__':
    print("Start swarm docker test")
    parser = argparse.ArgumentParser(description='Docker swarm evaluation tool.')
    parser.add_argument('config_path', metavar='config_path',
                    help='config_path for evaluation')
    
    args = parser.parse_args()
    with open(args.config_path, "r") as stream:
        config = yaml.safe_load(stream)
    pids, _ = run_swarm_docker_evaluation(config)

    try:
        time.sleep(10)
        sync_ctrl = SyncCtrl(rate=config["rate"], t_start=config["t_start"], duration=config["duration"], start_delay=0.1)
        sync_ctrl.work()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    
    for p in pids:
        kill_docker(p)
        pids[p].kill()