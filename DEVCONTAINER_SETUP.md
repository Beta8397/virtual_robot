# VS Code Dev Container Setup - Quick Start

## What's Configured

Your dev container is now configured with:
- ✅ BellSoft Liberica 17 Full JDK (with JavaFX)
- ✅ Desktop Lite feature for GUI applications
- ✅ VS Code Java extensions
- ✅ noVNC web-based desktop access

## Starting the Dev Container

1. **Open in VS Code**
   ```
   code .
   ```

2. **Reopen in Container**
   - VS Code will prompt you to "Reopen in Container"
   - Or use Command Palette (Cmd+Shift+P): "Dev Containers: Reopen in Container"

3. **Wait for Setup**
   - First time will take 5-10 minutes to:
     - Download base image
     - Install desktop environment
     - Install Liberica JDK 17 via SDKMAN
     - Install system dependencies

## Accessing the GUI Desktop

Once the container is running:

1. **Open Browser** to: http://localhost:6080
2. **Password**: `vscode`
3. **Run the application** from VS Code (F5) and it will appear in this desktop

## Running Virtual Robot

### Method 1: VS Code Debug (Recommended)
1. Press `F5` or go to Run & Debug
2. Select "Launch Virtual Robot"
3. View the GUI at http://localhost:6080

### Method 2: Terminal
```bash
cd /workspaces/virtual_robot
export JAVA_HOME=$HOME/.sdkman/candidates/java/current
java --module-path $JAVA_HOME/lib \
     --add-modules javafx.controls,javafx.fxml,javafx.graphics \
     -cp Controller/src:TeamCode/src \
     virtual_robot.controller.VirtualRobotApplication
```

## Verifying Installation

Run the task "Verify Java Installation" from VS Code:
- Command Palette (Cmd+Shift+P) → "Tasks: Run Task" → "Verify Java Installation"

Or manually:
```bash
java -version
java --list-modules | grep javafx
echo $DISPLAY
```

## Troubleshooting

**GUI not showing?**
- Ensure you're accessing http://localhost:6080
- Check that DISPLAY is set: `echo $DISPLAY` (should be `:1`)
- Restart the desktop: `sudo systemctl restart x11-common`

**Java not found?**
- Source SDKMAN: `source ~/.sdkman/bin/sdkman-init.sh`
- Check Java: `sdk current java`

## Next Steps

After the container starts successfully, you can:
1. Write OpModes in `TeamCode/src/org/firstinspires/ftc/teamcode`
2. Use @TeleOp or @Autonomous annotations
3. Run and test in the virtual robot simulator
4. Access the desktop at http://localhost:6080 to see the GUI

For more details, see `.devcontainer/README.md`
