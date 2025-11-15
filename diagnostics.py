#!/usr/bin/env python3
import sys, os, json, itertools, threading, time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime
from tqdm import tqdm

# ANSI color codes
RESET = "\033[0m"
BOLD = "\033[1m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RED = "\033[91m"

# Verbosity flags
QUIET = "--quiet" in sys.argv
VERBOSE = "--verbose" in sys.argv

def qprint(message, color=RESET):
    if not QUIET or VERBOSE:
        print(f"{color}{message}{RESET}")

def spinner_task(stop_event):
    spinner = itertools.cycle(['|', '/', '-', '\\'])
    while not stop_event.is_set():
        sys.stdout.write(f"\r{CYAN}Replaying... {next(spinner)}{RESET}")
        sys.stdout.flush()
        time.sleep(0.1)
    sys.stdout.write("\r" + " " * 20 + "\r")

def replay_log(filename="run_log.csv", save_video=False, save_gif=False, save_both=False, export_name=None, out_dir="."):
    # Dummy trajectory for demo
    xs = list(range(200))
    ys = [i**0.5 for i in xs]

    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=2)

    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        line.set_data(xs[:frame], ys[:frame])
        return line,

    ani = animation.FuncAnimation(fig, update, frames=len(xs), init_func=init, interval=50, blit=False)

    os.makedirs(out_dir, exist_ok=True)
    if export_name is None:
        export_name = datetime.now().strftime("run_%Y-%m-%d_%H-%M")
    base_path = os.path.join(out_dir, export_name)

    # Export with progress bars
    total_frames = len(xs)
    if save_video or save_both:
        qprint("Exporting MP4...", CYAN)
        for _ in tqdm(range(total_frames), desc="Saving MP4", unit="frame", disable=(QUIET and not VERBOSE)):
            pass
        ani.save(f"{base_path}.mp4", writer="ffmpeg", fps=20)
        qprint(f"Replay video saved as {base_path}.mp4", GREEN)

    if save_gif or save_both:
        qprint("Exporting GIF...", CYAN)
        for _ in tqdm(range(total_frames), desc="Saving GIF", unit="frame", disable=(QUIET and not VERBOSE)):
            pass
        ani.save(f"{base_path}.gif", writer="pillow", fps=10)
        qprint(f"Replay GIF saved as {base_path}.gif", GREEN)

    # Spinner during live replay
    if not QUIET or VERBOSE:
        stop_event = threading.Event()
        spinner_thread = threading.Thread(target=spinner_task, args=(stop_event,))
        spinner_thread.start()

    plt.show()

    if not QUIET or VERBOSE:
        stop_event.set()
        spinner_thread.join()
        qprint("Replay finished successfully!", GREEN)

def show_help():
    commands = f"""
{BOLD}{CYAN}Available commands:{RESET}

{GREEN}Replay & Export:{RESET}
  --replay             Run replay mode
  --save               Export replay as MP4
  --gif                Export replay as GIF
  --both               Export replay as both MP4 and GIF
  --name <filename>    Custom export name
  --out <directory>    Custom export directory

{YELLOW}Config Management:{RESET}
  --make-config        Create default config.json
  --show-config        Display current config.json settings
  --edit-config        Update config.json fields (use with --name, --out, --save, --gif, --both)
  --reset-config       Reset config.json to defaults
  --delete-config      Delete config.json completely
  --help-config        Show guide to config fields

{CYAN}General:{RESET}
  --list-commands      Show this help message
  --help               Alias for --list-commands
  --quiet              Suppress spinner/progress/colors
  --verbose            Force maximum detail

{BOLD}Usage Examples:{RESET}
  {GREEN}python3 robot_script.py --replay --save{RESET}
      → Replay and export as MP4 (auto timestamp name)

  {GREEN}python3 robot_script.py --replay --gif --name demo_run --out results/{RESET}
      → Replay and export as GIF named demo_run.gif in results/ folder

  {YELLOW}python3 robot_script.py --make-config{RESET}
      → Create default config.json

  {YELLOW}python3 robot_script.py --edit-config --gif --name experiment1{RESET}
      → Update config.json to enable GIF export and set name to experiment1

  {RED}python3 robot_script.py --reset-config{RESET}
      → Reset config.json to defaults

  {RED}python3 robot_script.py --delete-config{RESET}
      → Delete config.json completely
    """
    print(commands)

def show_config_help():
    guide = f"""
{BOLD}{CYAN}Config.json fields:{RESET}

  {YELLOW}export_name{RESET}   → Default filename for exports
  {YELLOW}out_dir{RESET}       → Default output directory
  {YELLOW}save_video{RESET}    → True/False, export MP4
  {YELLOW}save_gif{RESET}      → True/False, export GIF
  {YELLOW}save_both{RESET}     → True/False, export both formats
  {YELLOW}verbosity{RESET}     → "quiet" or "verbose" (default output style)
    """
    print(guide)

if __name__ == "__main__":
    if "--list-commands" in sys.argv or "--help" in sys.argv:
        show_help()

    elif "--help-config" in sys.argv:
        show_config_help()

    elif "--make-config" in sys.argv or "--reset-config" in sys.argv:
        default_config = {
            "export_name": "default_run",
            "out_dir": "logs",
            "save_video": True,
            "save_gif": False,
            "save_both": False,
            "verbosity": "verbose"
        }
        with open("config.json", "w") as f:
            json.dump(default_config, f, indent=4)
        if "--make-config" in sys.argv:
            qprint("Default config.json created with settings:", CYAN)
        else:
            qprint("Config.json reset to defaults:", CYAN)
        print(json.dumps(default_config, indent=4))

    elif "--show-config" in sys.argv:
        if os.path.exists("config.json"):
            with open("config.json", "r") as f:
                cfg = json.load(f)
            qprint("Current config.json settings:", CYAN)
            print(json.dumps(cfg, indent=4))
        else:
            qprint("No config.json found. Run with --make-config to create one.", RED)

    elif "--edit-config" in sys.argv:
        if os.path.exists("config.json"):
            with open("config.json", "r") as f:
                cfg = json.load(f)
            for i, arg in enumerate(sys.argv):
                if arg == "--name" and i+1 < len(sys.argv):
                    cfg["export_name"] = sys.argv[i+1]
                elif arg == "--out" and i+1 < len(sys.argv):
                    cfg["out_dir"] = sys.argv[i+1]
                elif arg == "--save":
                    cfg["save_video"] = True
                elif arg == "--gif":
                    cfg["save_gif"] = True
                elif arg == "--both":
                    cfg["save_both"] = True
            with open("config.json", "w") as f:
                json.dump(cfg, f, indent=4)
            qprint("Config.json updated with new settings:", YELLOW)
            print(json.dumps(cfg, indent=4))
        else:
            qprint("No config.json found. Run with --make-config to create one.", RED)

    elif "--delete-config" in sys.argv:
        if os.path.exists("config.json"):
            os.remove("config.json")
            qprint("Config.json has been deleted.", RED)
        else:
            qprint("No config.json found to delete.", RED)

    elif "--replay" in sys.argv:
        # Flags from CLI
        save_flag = "--save" in sys.argv
        gif_flag = "--gif" in sys.argv
        both_flag = "--both" in sys.argv
        export_name = None
        out_dir = "."

        # Load config defaults if present
        if os.path.exists("config.json"):
            with open("config.json", "r") as f:
                cfg = json.load(f)
            export_name = cfg.get("export_name", export_name)
            out_dir = cfg.get("out_dir", out_dir)
            save_flag = cfg.get("save_video", save_flag)
            gif_flag = cfg.get("save_gif", gif_flag)
            both_flag = cfg.get("save_both", both_flag)

            # Apply verbosity from config only if no CLI override
            if not QUIET and not VERBOSE:
                v = cfg.get("verbosity")
                if v == "quiet":
                    QUIET = True
                elif v == "verbose":
                    VERBOSE = True

        # CLI overrides take precedence
        if "--name" in sys.argv:
            idx = sys.argv.index("--name")
            if idx + 1 < len(sys.argv):
                export_name = sys.argv[idx + 1]
        if "--out" in sys.argv:
            idx = sys.argv.index("--out")
            if idx + 1 < len(sys.argv):
                out_dir = sys.argv[idx + 1]

        # Resolve incompatible flags: --both overrides --save/--gif
        if both_flag:
            save_flag, gif_flag = True, True

        # Info print (verbose only)
        qprint(
            f"Replay settings → save_video={save_flag}, save_gif={gif_flag}, "
            f"out_dir={out_dir}, name={export_name}",
            CYAN
        )

        # Run replay
        replay_log(
            save_video=save_flag,
            save_gif=gif_flag,
            save_both=both_flag,
            export_name=export_name,
            out_dir=out_dir
        )
