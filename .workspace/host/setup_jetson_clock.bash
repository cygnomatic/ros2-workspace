#!/bin/bash

SERVICE_FILE="/etc/systemd/system/jetson_clocks.service"

read -r -d '' SERVICE_CONTENT << EOF
[Unit]
Description=Run jetson_clocks at boot
After=multi-user.target
[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
User=root
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
EOF

echo "${SERVICE_CONTENT}" | sudo tee ${SERVICE_FILE} > /dev/null

sudo systemctl daemon-reload
sudo systemctl enable jetson_clocks.service

echo "jetson_clocks service has been set up and enabled to start at boot."