To access the Raspberry Pi over the network:

1. **Physical Connection**
   - Connect the **WAN port** of the router to the Raspberry Pi.
   - Power on the Pi.
   - Power on the Router.

2. **Connect Your Laptop**
   - Join the Wi-Fi network for the router:
     - **SSID:** `lunabotics`
     - **Password:** `lunabotics@osu`

3. **Access the Pi via SSH**
   - Open a terminal or use the **VSCode Remote - SSH** extension.
   - Run:
     ```bash
     ssh fros@192.168.0.10
     ```
   - **Password:** `lunabotics2024`
   - The Pi uses a **static IP**, so this address should not change.

4. **Troubleshooting**
   - Ensure all cables are connected properly and try a different Ethernet cable if needed.
   - If SSH fails:
     1. Log in to the router web interface (usually at `192.168.0.1` or `192.168.1.1`), on your browser.
     2. Make sure the router IP is set to `192.168.0.1` (change if needed).
     3. Verify the **DHCP range** includes the Pi’s IP (`192.168.0.10`).
   - After these changes, restart the router, reconnect your laptop and try SSH again.
