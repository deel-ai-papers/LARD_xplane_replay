# Before starting this script, make sure to start an xplane flight at any airport
# ALL EXISTING SCREENSHOTS ARE ERASED FROM THE SCREENSHOT DIRECTORY

import argparse
import glob
import os
import re
import time

import yaml
from PIL import Image

from XPlaneConnectX import XPlaneConnectX


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_FILE = os.path.join(SCRIPT_DIR, "runner_config.yaml")


def local_to_zulu(local_time, longitude):
    # convert local time to zulu time (UTC) based on longitude only.
    # Parameters:
    #   local_time in hours
    #   longitude in degrees

    # Return
    #   zulu time (UTC) in hours
    time_difference = longitude / 15.0
    zulu_time = local_time - time_difference

    if zulu_time < 0:
        zulu_time += 24
    elif zulu_time >= 24:
        zulu_time -= 24

    return zulu_time


def remove_files(pattern):
    files = glob.glob(pattern)
    for file in files:
        try:
            os.remove(file)
            print(f"Removed: {file}")
        except Exception as error:
            print(f"Error removing {file}: {error}")


# Called at the end of the scenario, to resize, rename and move the screenshots in the scenario dependant directory
def process_images(input_folder, output_folder, scenario_name, nb_digits, width, height):
    scenario_folder = os.path.join(output_folder, scenario_name)
    os.makedirs(scenario_folder, exist_ok=True)
    images = sorted(
        file
        for file in os.listdir(input_folder)
        if file.lower().endswith(("png", "jpg", "jpeg"))
    )

    for idx, image_name in enumerate(images, start=1):
        image_path = os.path.join(input_folder, image_name)
        with Image.open(image_path) as img:
            img_width, img_height = img.size
            left = (img_width - width) // 2
            top = (img_height - height) // 2
            right = left + width
            bottom = top + height

            cropped_img = img.crop((left, top, right, bottom))
            extension = os.path.splitext(image_name)[1].lower()
            new_image_name = f"{scenario_name}_{idx - 1:0{nb_digits}d}{extension}"
            cropped_img.save(os.path.join(scenario_folder, new_image_name))

        os.remove(image_path)


def resolve_path(path, base_directory):
    """Resolve relative paths from the config file directory."""
    if os.path.isabs(path) or re.match(r"^[A-Za-z]:[\\/]", path):
        return os.path.normpath(path)
    return os.path.normpath(os.path.join(base_directory, path))


def load_config(config_file):
    config_file = os.path.abspath(config_file)
    try:
        with open(config_file, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"Cannot load config file '{config_file}': {error}") from error

    required_sections = ("xplane", "paths", "capture", "delays", "default_time")
    missing_sections = [section for section in required_sections if section not in config]
    if missing_sections:
        raise ValueError(f"Missing config section(s): {', '.join(missing_sections)}")

    config_directory = os.path.dirname(config_file)
    for key in ("scenario_directory", "output_directory", "screenshot_directory"):
        config["paths"][key] = resolve_path(config["paths"][key], config_directory)

    return config


# Traverse all the poses of the scenario described in the yaml input file
# - Set up the position and attitude
# - Take a screenshot
# - Resize, rename and move the screenshots in a scenario dependant directory
def run_scenario(input_file, config):
    print(f"Capturing scenario {input_file}...")

    # Connect to Xplane. Assumes X-Plane runs on the same machine and uses 
    # the default port 49000 for UDP. Values are configurable in runner_config.yaml.
    xplane_config = config["xplane"]
    xpc = XPlaneConnectX(ip=xplane_config["ip"], port=xplane_config["port"])

    # subscribe to datarefs (necessary prior to use sendDREF ?)
    subscribed_drefs = [
        ("sim/flightmodel/position/groundspeed", 1),  # ground speed in m/s at 1Hz
    ]
    xpc.subscribeDREFs(subscribed_drefs)  # the current values are stored in xpc.current_dref_values

    paths = config["paths"]
    capture = config["capture"]
    delays = config["delays"]
    default_time = config["default_time"]

    screenshot_filter = os.path.join(paths["screenshot_directory"], "*")

    # delete all the files from screenshot directory that correspond to the pattern (allow to redo the numbering in next scenario)
    remove_files(screenshot_filter)

    try:
        with open(input_file, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as error:
        print("####ERROR##### input file error", error)
        return False

    if not isinstance(data, dict) or not isinstance(data.get("poses"), list):
        print("####ERROR##### input file must contain a 'poses' list")
        return False

    filename = os.path.splitext(os.path.basename(input_file))[0]
    print(f"{filename} loaded!")

    # Extract camera size from YAML
    entry = data.get("image", {})
    campic_res_x_pix = entry.get("width", capture["default_width"])
    campic_res_y_pix = entry.get("height", capture["default_height"])
    xpc.sendDREF("sim/graphics/view/window_width", campic_res_x_pix)  # WARNING: this DREF DOESN'T SEEM TO WORK. NO PROBLEM, WE ADAPT THE FOV LATER
    xpc.sendDREF("sim/graphics/view/window_height", campic_res_y_pix)
    set_campic_res_y_pix = xpc.getDREF("sim/graphics/view/window_height")
    set_campic_res_x_pix = xpc.getDREF("sim/graphics/view/window_width")
    print(f"Prog camera size (width={set_campic_res_x_pix}, height={set_campic_res_y_pix})")

    fact_width = set_campic_res_x_pix / campic_res_x_pix
    fact_height = set_campic_res_y_pix / campic_res_y_pix

    # If the xplane window is smaller than the target crop image we raise an error
    if fact_width < 1.0 or fact_height < 1.0:
        print(
            "####ERROR##### The xplane window size shall be at least "
            f"{campic_res_x_pix}x{campic_res_y_pix}"
        )
        return False

    # set the Field Of View
    # The field of view(s) in LARD V2.0 scenarios correspond to a window size of 1024x1024
    # We adapt the programmed field of views to get the desired one after the images crop
    default_fov = capture["default_fov"]
    campic_fov_h_deg = entry.get("fov", entry.get("fov_x", default_fov))
    campic_fov_v_deg = entry.get("fov", entry.get("fov_y", default_fov))  # useful only if next is True
    non_proportional_vertical_fov = capture["non_proportional_vertical_fov"]

    print(f"Desired Fov (horizontal={campic_fov_h_deg}, vertical={campic_fov_v_deg})")

    prog_fov_h_deg = fact_width * campic_fov_h_deg
    prog_fov_v_deg = fact_height * campic_fov_v_deg

    xpc.sendDREF(
        "sim/graphics/settings/non_proportional_vertical_FOV",
        non_proportional_vertical_fov,
    )

    print("setting fov h =", prog_fov_h_deg)
    xpc.sendDREF("sim/graphics/view/field_of_view_deg", prog_fov_h_deg)

    if non_proportional_vertical_fov:
        print("setting fov v =", prog_fov_v_deg)
        xpc.sendDREF("sim/graphics/view/vertical_field_of_view_deg", prog_fov_v_deg)
    else:
        print("setting same for fov v =", prog_fov_h_deg)  # already set by the sendDREF of field_of_view_deg

    # remove all equipment from the scene before screenshot
    xpc.sendCMND("sim/view/forward_with_nothing")

    # Prepare output directories
    output_dir = os.path.join(paths["output_directory"], filename)
    os.makedirs(output_dir, exist_ok=True)

    # Handling number of digits for the file numbering
    num_digits = max(1, len(str(len(data["poses"]) - 1)))  # Number of digits of the number of total images

    previous_airport = None
    previous_runway = None

    xpc.pauseSIM(True)  # for "long jumps" you want to pause the simulator

    # full_filename = None
    for i, entry in enumerate(data["poses"]):
        pose = entry.get("pose", [])
        if len(pose) < 6:
            print("Skipping incomplete pose data ", i)
            continue

        lon, lat, alt, heading, pitch, bank = (
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
        )

        # Set the position location
        xpc.sendPOSI(
            lat=lat,  # latitude in degrees
            lon=lon,  # longitude in degrees
            elev=alt,  # altitude above mean sea level in meters
            phi=bank,  # roll angle in degrees
            theta=pitch - 90,  # pitch angle in degrees (vertical-90)
            psi_true=heading,  # true (not magnetic) heading
        )

        # Set the max time to wait for loading
        current_airport = entry.get("airport", "")
        current_runway = entry.get("runway", "")

        print(f"Airport {current_airport} / {current_runway}")

        current_date = entry.get("time") or {}
        current_month = current_date.get("month", default_time["month"])
        current_day = current_date.get("day", default_time["day"])
        current_hour = current_date.get("hour", default_time["hour"])
        current_hour = local_to_zulu(current_hour, lon)
        current_minute = current_date.get("minute", default_time["minute"])

        xpc.sendDREF("sim/time/local_date_days", (current_month - 1) * 30 + current_day)
        xpc.sendDREF("sim/time/zulu_time_sec", current_hour * 3600 + current_minute * 60)

        if current_airport != previous_airport:
            time.sleep(delays["airport_change_seconds"])  # >35 seconds recommended for airport change
            previous_airport = current_airport
            previous_runway = current_runway
        elif current_runway != previous_runway:
            time.sleep(delays["runway_change_seconds"])  # >3 seconds recommended for runway change within the same airport
            previous_runway = current_runway
        else:
            time.sleep(delays["same_runway_seconds"])  # 1 second for same runway

        xpc.sendCMND("sim/operation/screenshot")

        # waiting for the screenshot to be taken, before skipping to the next scene.
        time.sleep(delays["screenshot_seconds"])

        alt_agl = xpc.getDREF("sim/graphics/view/view_elevation_agl_mtrs")
        alt_msl = xpc.getDREF("sim/graphics/view/view_elevation_msl_mtrs")

        ground_level = alt_msl - alt_agl
        print(
            f"Ground level under aircraft for lat/lon {lat},{lon} msl {alt_msl} "
            f"agl {alt_agl} ground level {ground_level}"
        )

    # At the very end, copy all screenshots in the target folder
    process_images(
        input_folder=paths["screenshot_directory"],  # Path of the xplane screenshots folder
        output_folder=paths["output_directory"],  # Folder to store generated images
        scenario_name=filename,  # Scenario name
        nb_digits=num_digits,
        width=campic_res_x_pix,
        height=campic_res_y_pix,
    )
    return True


def close_xplane(window_title):
    try:
        import pygetwindow as gw

        windows = gw.getWindowsWithTitle(window_title)
        if not windows:
            print(f"####ERROR##### No window found with title '{window_title}'")
            return
        print(f"Closing window {windows[0]}")
        windows[0].close()
    except Exception as error:
        print(f"####ERROR##### Could not close X-Plane: {error}")


def process_yaml_files(folder_path, config, close_after_gen=False):
    try:
        yaml_files = sorted(
            filename for filename in os.listdir(folder_path) if filename.endswith(".yaml")
        )
        if not yaml_files:
            print(f"####ERROR##### No YAML scenario found in '{folder_path}'")
            return False

        success = True
        for filename in yaml_files:
            file_path = os.path.join(folder_path, filename)
            success = run_scenario(file_path, config) and success
        return success
    except OSError as error:
        print(f"####ERROR##### Error during generation: {error}")
        return False
    finally:
        if close_after_gen:
            close_xplane(config["xplane"]["window_title"])


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Replay LARD YAML scenarios in X-Plane and capture screenshots."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help="Runner configuration file (default: runner_config.yaml next to this script).",
    )
    input_group = parser.add_mutually_exclusive_group()
    input_group.add_argument("--scenario", help="Run one YAML scenario file.")
    input_group.add_argument("--scenario-dir", help="Run every YAML file from this directory.")
    parser.add_argument(
        "--close-after-generation",
        action="store_true",
        help="Close the configured X-Plane window after processing.",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    try:
        config = load_config(args.config)
    except ValueError as error:
        print(f"####ERROR##### {error}")
        return 1

    close_after_gen = args.close_after_generation or config["xplane"].get(
        "close_after_generation", False
    )

    if args.scenario:
        scenario = resolve_path(args.scenario, os.getcwd())
        success = run_scenario(scenario, config)
        if close_after_gen:
            close_xplane(config["xplane"]["window_title"])
    else:
        scenario_directory = (
            resolve_path(args.scenario_dir, os.getcwd())
            if args.scenario_dir
            else config["paths"]["scenario_directory"]
        )
        success = process_yaml_files(scenario_directory, config, close_after_gen)

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())