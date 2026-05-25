# Virtual Robot Dev Container Setup

This dev container provides a complete development environment for the Virtual Robot Simulator with:
- **Java 17** (BellSoft Liberica Full JDK with JavaFX)
- **Desktop environment** (accessible via noVNC on port 6080)
- **VS Code Java extensions**

## Getting Started

1. **Open in Dev Container**
   - Open this folder in VS Code
   - When prompted, click "Reopen in Container" (or use Command Palette: "Dev Containers: Reopen in Container")
   - Wait for the container to build and setup to complete

2. **Access the Desktop Environment**
   - Once the container is running, open your browser to: `http://localhost:6080`
   - Password: `vscode`
   - This provides a full desktop environment where GUI applications will display

3. **Run the Virtual Robot Application**
   - In VS Code, go to Run and Debug (Cmd+Shift+D)
   - Select "Launch Virtual Robot" from the dropdown
   - Click the green play button or press F5
   - The application will open in the desktop environment (access via browser on port 6080)

## Alternative: Running from Terminal

You can also run the application from the integrated terminal:

```bash
java --module-path $JAVA_HOME/lib \
     --add-modules javafx.controls,javafx.fxml,javafx.graphics \
     -cp Controller/src:TeamCode/src:lib/* \
     virtual_robot.controller.VirtualRobotApplication
```

## Project Structure

- `Controller/src` - Main application code
- `TeamCode/src` - User-defined OpModes
- `lib/` - External dependencies

## Notes

- The DISPLAY environment variable is automatically set to `:1`
- JavaFX is included in the Liberica Full JDK
- Additional graphics libraries are pre-installed for GUI support
