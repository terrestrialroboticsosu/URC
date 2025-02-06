#!/bin/sh

# Function to install packages using apt (Debian/Ubuntu)
install_with_apt() {
    sudo apt-get update
    sudo apt-get install -y build-essential gdb git pkg-config meson ninja-build python3 cmake
}

# Function to install packages using yum (Red Hat/CentOS)
install_with_yum() {
    sudo yum groupinstall -y "Development Tools"
    sudo yum install -y gdb meson cmake pkgconfig git python3 ninja-build
}

# New package manager used in Fedora
install_with_dnf() {
    sudo dnf groupinstall -y "Development Tools"
    sudo dnf install -y gdb meson ninja-build python3 cmake pkgconfig git
}

# Function to install packages using pacman (Arch Linux)
install_with_pacman() {
    sudo pacman -Sy --noconfirm base-devel gdb meson ninja python3 cmake pkgconf git
}

if [ -f /etc/arch-release ]; then
    install_with_pacman
elif [ -f /etc/debian_version ]; then
    install_with_apt
elif [ -f /etc/redhat-release ] || [ -f /etc/centos-release ]; then
    if command -v dnf >/dev/null 2>&1; then
        install_with_dnf
    else
        install_with_yum
    fi
else
    echo "Your linux distro is not supported currently."
    echo "You need to manually install those packages: exiftool, jq, glfw"
fi

# Check if /usr/local/lib is already in /etc/ld.so.conf
if ! grep -q "^/usr/local/lib$" /etc/ld.so.conf; then
    echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf > /dev/null
    # Update the library cache
    sudo ldconfig
fi

echo "====================="
echo "INSTALLATION FINISHED"
echo "====================="
