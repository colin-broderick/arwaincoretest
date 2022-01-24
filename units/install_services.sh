sudo systemctl disable --now arwain_controller.service
sudo systemctl disable --now arwain_dashboard.service
sudo cp ./units/* /etc/systemd/system/
sudo systemctl enable --now arwain_controller.service
sudo systemctl enable --now arwain_dashboard.service
