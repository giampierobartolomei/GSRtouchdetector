# AWE-POD • Soft-robotic goosebumps + GSR touch sensing

AWE-POD is a soft-robotic skin that inflates to evoke a spiral “goosebumps” effect while sensing **autonomic touch responses** via **GSR** (electrodermal activity).  
Goal: **detect touch** and related arousal from the nervous system using GSR, and optionally actuate pneumatic feedback.

![AWE-POD Prototype](IMG_5433.jpg)

## What this repository provides
- **Data acquisition** with the **Seeed Studio Grove GSR** module  
  https://wiki.seeedstudio.com/Grove-GSR_Sensor/
- **Final firmware** to sense GSR and actuate a micropump.
- **Offline analysis** tools to filter, extract features, train models, and convert trained models into C# for embedded deployment.
- **Mechanical mold** to cast the silicone spiral actuator that can be mounted in a bracelet or embedded in eco-leather.

## Repository layout
```
AWE-POD/
├─ dataset/                       # example GSR sessions (CSV)
├─ Fritzing_files/                # wiring/schematics (see files, no extra README detail)
├─ gsrgroove_datasaving&calibration/   # Grove GSR data logging + calibration
├─ handmade_electronics_code/     # archived "handmade" electronics code (not current)
├─ pump_test/                     # simple pump driver tests
├─ python_offline_analysis/       # offline filters, features, model training, C# export
├─ sense_and_actuate/             # FINAL firmware for the prototype
└─ stl files/                     # 3D files, includes awepod1.0.stl
```

### Hardware notes
- Use the Fritzing sources in `Fritzing_files/` for connections.  
- A micropump is driven by a transistor with a flyback diode. Power decoupling on the 5 V rail.  
- GSR sensing is via **Grove GSR**. Keep supply consistent with your previous data.

## The actuator: `awepod1.0.stl`
- A silicone mold that creates a **spiral channel**. When inflated, the pad expands laterally with a visible spiral effect.  
- Integration: fits a **bracelet** geometry in `stl files/` or can be **embedded in eco-leather** panels.

---

## Quick start

### 1) Acquire data with Grove GSR
Folder: `gsrgroove_datasaving&calibration/`  
- Flash the sketch to your **ESP32** board.  
- Start the serial logger on your PC to save CSV.  
- Use the included calibration routine before recording.  
Output CSV format (recommended):
```
timestamp_ms,adc_raw,gsr_uS,flag_touch,pump_state
```

### 2) Run the final firmware (sense + actuation)
Folder: `sense_and_actuate/`  
- Builds for **ESP32** (Arduino IDE or PlatformIO).  
- Reads GSR, applies real-time filtering and touch detection, controls the pump, and can stream or log data.

### 3) Test only the pump (optional)
Folder: `pump_test/`  
- Minimal firmware to verify transistor, diode, and supply under load.

### 4) Offline analysis and model training
Folder: `python_offline_analysis/`  
- Create a virtual environment and install dependencies:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
- Use the notebook or scripts to:
  - low-pass filter GSR (e.g., 0.5 Hz),
  - compute SCL/EDA features,
  - train classifiers on labeled events,
  - export trained models to **C#** for on-device inference.

---

## Data and workflow

1. **Record** with `gsrgroove_datasaving&calibration/` and store sessions in `dataset/`.  
2. **Prototype testing** with `sense_and_actuate/` to couple sensing and pneumatic feedback.  
3. **Analyze + train** in `python_offline_analysis/`.  
4. **Iterate** on actuator design using `stl files/awepod1.0.stl` and variants.

---

## Status of electronics tracks
- **Grove GSR–based path**: **current and recommended** for data collection and prototype validation.  
- **handmade_electronics_code**: **archived**. Kept for reference only.

---

## License
Add a license file (e.g., MIT for code; consider a permissive license or CC-BY for CAD/STL).

## Citation
If you build on AWE-POD in publications or demos, please cite related work in your profile and link back to this repository.
