#!/bin/bash
set -e

echo "Setting up Virtual Robot development environment..."

# Install basic dependencies first
echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
    curl \
    zip \
    unzip \
    libxrender1 \
    libxtst6 \
    libxi6 \
    libgtk-3-0 \
    x11-apps \
    libgl1-mesa-dri \
    libglx0 || true

# Try to install libgl1-mesa-glx if available (older Debian versions)
sudo apt-get install -y libgl1-mesa-glx 2>/dev/null || true

# Install SDKMAN if not already installed
if [ ! -d "$HOME/.sdkman" ]; then
    echo "Installing SDKMAN..."
    curl -s "https://get.sdkman.io" | bash
fi

# Source SDKMAN
export SDKMAN_DIR="$HOME/.sdkman"
[[ -s "$HOME/.sdkman/bin/sdkman-init.sh" ]] && source "$HOME/.sdkman/bin/sdkman-init.sh"

# Install Liberica 17 Full JDK (includes JavaFX)
echo "Installing BellSoft Liberica 17 Full JDK..."
sdk install java 17.0.13.fx-librca || sdk use java 17.0.13.fx-librca || true
sdk default java 17.0.13.fx-librca || true

# Verify installation
echo "Java version:"
java -version

# Set JAVA_HOME permanently
export JAVA_HOME="$HOME/.sdkman/candidates/java/current"
if ! grep -q "JAVA_HOME" "$HOME/.bashrc"; then
    echo "export JAVA_HOME=$HOME/.sdkman/candidates/java/current" >> $HOME/.bashrc
    echo "export PATH=\$JAVA_HOME/bin:\$PATH" >> $HOME/.bashrc
fi

# Also add to .bash_profile for login shells
if ! grep -q "JAVA_HOME" "$HOME/.bash_profile"; then
    echo "export JAVA_HOME=$HOME/.sdkman/candidates/java/current" >> $HOME/.bash_profile
    echo "export PATH=\$JAVA_HOME/bin:\$PATH" >> $HOME/.bash_profile
fi

echo "Setup complete! Java environment is ready."
echo "Java Home: $JAVA_HOME"
echo "Desktop environment will be available on port 6080 (noVNC)."
echo "You can access it at: http://localhost:6080"
echo ""
echo "To verify JavaFX is available, run: java --list-modules | grep javafx"
