sudo cp ./units/* /etc/systemd/system/
sudo systemctl enable arwain_controller.service
sudo systemctl start arwain_controller.service
sudo systemctl enable arwain_dashboard.service
sudo systemctl start arwain_dashboard.service
