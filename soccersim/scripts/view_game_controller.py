#!/usr/bin/env python3
import sys
import json
import shutil
import subprocess

def format_data(data):
    lines = []
    lines.append("=" * 66)
    lines.append("                      ⚽ ROBOCUP GAME DASHBOARD ⚽")
    lines.append("=" * 66)

    # Extract raw field values.
    game_phase = data.get("gamePhase", "N/A")
    state = data.get("state", "N/A")
    set_play = data.get("setPlay", "N/A")
    stopped = str(data.get("stopped", "N/A")).upper()
    kicking_team = data.get("kickingTeam", "N/A")
    pkt_num = data.get("packetNumber", "N/A")

    # Time-related fields.
    first_half = str(data.get("firstHalf", "N/A")).upper()
    secs_remaining = data.get("secsRemaining", "N/A")
    secondary_time = data.get("secondaryTime", "N/A")

    # Two-column aligned layout: five fields on the left, four on the right.
    lines.append(f"  gamePhase:     {game_phase:<16} | setPlay:      {set_play:<16}")
    lines.append(f"  state:         {state:<16} | kickingTeam:  {kicking_team:<16}")
    lines.append(f"  firstHalf:     {first_half:<16} | stopped:      {stopped:<16}")
    lines.append(f"  secsRemaining: {secs_remaining:<16} | packetNumber: {pkt_num:<16}")
    lines.append(f"  secondaryTime: {secondary_time:<16} |")
    lines.append("=" * 66)
    lines.append("")

    for team in data.get("teams", []):
        team_num = team.get("teamNumber", 0)
        score = team.get("score", 0)
        budget = team.get("messageBudget", 0)
        color = "Blue" if team.get("fieldPlayerColour") == 0 else "Red"

        lines.append(f" 🚩 TEAM {team_num} ({color}) [score: {score} | messageBudget: {budget}]")
        lines.append("-" * 66)
        lines.append(f"  {'ID':<6} | {'penalty':<12} | {'secsTillUnpenalised':<20} | {'warnings':<8} | {'cautions':<8}")
        lines.append("-" * 66)

        players = team.get("players", [])[:3]
        for i, p in enumerate(players, start=1):
            penalty = p.get("penalty", "NONE")
            secs_free = p.get("secsTillUnpenalised", 0)
            warn = p.get("warnings", 0)
            caut = p.get("cautions", 0)
            lines.append(f"  #{i:<4} | {penalty:<12} | {secs_free:<20} | {warn:<8} | {caut:<8}")

        lines.append("-" * 66)
        lines.append("")

    return "\n".join(lines)

def main():
    cmd = ["ros2", "topic", "echo", "/soccer/game_controller", "--field", "data"]

    # Start the ROS 2 topic listener subprocess and read lines in real time.
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)

    try:
        # Continuously read output lines from the subprocess.
        for line in iter(process.stdout.readline, ""):
            line = line.strip()
            if not line:
                continue
            try:
                if line.startswith('"') and line.endswith('"'):
                    line = json.loads(line)
                data = json.loads(line)

                output = format_data(data)

                # Check terminal height before rendering to avoid scrolling.
                term_height = shutil.get_terminal_size().lines
                lines = output.splitlines()
                if len(lines) > term_height - 1:
                    lines = lines[:term_height - 2] + [f'... [由于数据行数（{len(lines)}）超出终端高度（{term_height}），已安全截断]']

                final_output = '\n'.join(lines)
                sys.stdout.write("\033[H\033[J" + final_output + "\n")
                sys.stdout.flush()
            except Exception:
                pass
    except KeyboardInterrupt:
        # Terminate the subprocess cleanly when the user exits with Ctrl+C.
        process.terminate()
        print("\n[INFO] 已停止查看 GameController 状态。")
    finally:
        # Ensure the subprocess is stopped.
        try:
            process.kill()
        except OSError:
            pass

if __name__ == "__main__":
    main()
