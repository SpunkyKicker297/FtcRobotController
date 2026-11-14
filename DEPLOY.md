# Deploying TeamCode to the REV Control Hub

This guide explains how to build our team's code and upload it to the robot's
REV Control Hub directly from the IDE's terminal — no Android Studio required.

You only need to do the **one-time setup** once per computer. After that, the
**deploy steps** are what you run each time you want to put new code on the robot.

---

## One-time setup (per computer)

The Android SDK ships with `adb` (Android Debug Bridge), the tool that talks to
the Control Hub. It lives at `~/Library/Android/sdk/platform-tools/adb`. We add
that folder to the shell `PATH` so we can just type `adb`.

This line is in `~/.zshrc`:

```bash
export PATH="$PATH:$HOME/Library/Android/sdk/platform-tools"
```

After adding it, open a new terminal (or run `source ~/.zshrc`) and confirm:

```bash
adb version
```

You should see `Android Debug Bridge version 1.0.41` (or similar).

---

## Deploy steps (every time)

### 1. Build the code

From the project root, build the TeamCode app:

```bash
./gradlew :TeamCode:assembleDebug
```

A successful build ends with `BUILD SUCCESSFUL` and produces the app at:

```
TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
```

> Tip: You can also skip ahead and use `installDebug` (step 4) which builds
> *and* installs in one command. Building separately first is a good way to
> catch code errors before you're standing at the robot.

### 2. Connect to the Control Hub's Wi-Fi

On your computer, join the Control Hub's Wi-Fi network:

- Network name (SSID): looks like `FTC-xxxx` (printed on the hub / set during config)
- Password: the one set when the hub was configured

Your laptop is now on the robot's network. Note: while connected to the hub you
won't have normal internet access, so build (step 1) BEFORE connecting if your
build needs to download anything.

### 3. Point adb at the Control Hub

The Control Hub is always reachable at `192.168.43.1` on port `5555`:

```bash
adb connect 192.168.43.1:5555
```

Confirm the hub shows up as `device` (not `offline`):

```bash
adb devices
```

Expected output:

```
List of devices attached
192.168.43.1:5555   device
```

If it says `offline` or nothing appears, you're probably not on the hub's Wi-Fi
yet — re-check step 2.

### 4. Install the app onto the hub

```bash
./gradlew :TeamCode:installDebug
```

This builds (if needed) and pushes the Robot Controller app — including our
TeamCode OpModes — onto the Control Hub. This replaces the app currently on the
hub, which is the normal FTC deploy. Don't do it mid-match.

### 5. Run it from the Driver Station

On the Driver Station, our OpModes now appear in the menu:

- **TeleOp Drive** (group: Competition) — driver control
- **Auto: Park in Loading Zone** (group: Competition) — 30-second autonomous
- **Motor Test v2: Intake & Shooter** (group: Test) — mechanism test

---

## Control reference

### TeleOp Drive — two controllers

**gamepad1 (driver) — driving and primary intake**

| Control        | Action                              |
|----------------|-------------------------------------|
| Left stick Y   | Drive forward / backward            |
| Left stick X   | Strafe left / right                 |
| Right stick X  | Turn (yaw)                          |
| Right trigger  | Hold for turbo (full speed)         |
| Right bumper   | Run primary intake IN (hold)        |
| Left bumper    | Run primary intake OUT / eject (hold) |

**gamepad2 (shooter operator)**

| Control        | Action                                          |
|----------------|-------------------------------------------------|
| A button       | Start / stop shooter sequence (spin up, open gate, feed) |

---

## Troubleshooting

- **`adb: command not found`** — The `PATH` line isn't loaded in this terminal.
  Run `source ~/.zshrc` or open a new terminal.
- **`adb devices` shows `offline` / nothing** — Not connected to the hub's Wi-Fi,
  or the hub isn't powered on. Reconnect and try `adb connect 192.168.43.1:5555`.
- **App crashes at INIT with "cannot find a hardware device"** — A motor name in
  the code doesn't match the Robot Configuration. Check `front_left_drive`,
  `back_left_drive`, `front_right_drive`, `back_right_drive`, `intake`, `shooter`.
- **A motor spins the wrong way** — Flip that motor's `setDirection(...)` in
  `RobotHardware.init()` from `FORWARD` to `REVERSE` (or vice-versa).
