from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
import subprocess
import os
from typing import Optional

app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class Command(BaseModel):
    command: str  # "start" | "end"
    map_name: Optional[str] = None  # ← 이 줄 추가

# 실행 중인 프로세스 저장
processes = {
    "slam": None,
    "nav": None,
}

def run_shell_script_in_xterm(name: str, mapid: str | None = None):
    ros_ws = os.path.expanduser("./")
    if name == "slam":
        script_path = "start_slam.sh"
        args = []
    elif name == "nav":
        script_path = "start_nav.sh"
        if not mapid:
            raise ValueError("mapid is required for nav")
        args = [mapid]
    else:
        raise ValueError("Unknown launch name")

    abs_script_path = os.path.join(ros_ws, script_path)
    env = os.environ.copy()

    # xterm 명령어 구성
    # -hold : 스크립트 종료 후 터미널 닫히지 않음
    # -e : 실행할 명령어
    cmd = [
        "xterm",
        "-hold",
        "-e",
        f"bash '{abs_script_path}' {' '.join(args)}"
    ]

    proc = subprocess.Popen(
        cmd,
        cwd=ros_ws,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return proc

def control_launch(name: str, command: str, mapid: str | None = None):
    global processes

    if command == "start":
        if processes[name] is None or processes[name].poll() is not None:
            try:
                processes[name] = run_shell_script_in_xterm(name, mapid=mapid)
                return {"status": f"{name} launch started in new xterm"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        else:
            return {"status": f"{name} is already running"}

    elif command == "end":
        if processes[name] and processes[name].poll() is None:
            processes[name].terminate()
            processes[name].wait(timeout=10)
            processes[name] = None
            return {"status": f"{name} launch stopped"}
        else:
            return {"status": f"{name} is not running"}

    else:
        raise HTTPException(status_code=400, detail="Invalid command")

@app.post("/api/slam")
def control_slam(cmd: Command):
    result = control_launch("slam", cmd.command, mapid=cmd.map_name)
    print("수신된 명령:", cmd)
    # 'slam end'일 때 mapid가 존재하면 저장 로직 수행
    if cmd.command == "end" and cmd.map_name:
        map_name = cmd.map_name
        # 예: ROS service call 또는 파일 이동 등

        result["map_saved_as"] = map_name

    return result


@app.post("/api/nav")
def control_nav(cmd: Command):
    return control_launch("nav", cmd.command, mapid=cmd.map_name)
