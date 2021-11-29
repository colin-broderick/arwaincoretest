screen -ls | grep '(Detached)' | awk '{print $1}' | xargs -I % -t screen -X -S % quit
screen -dm -S control_panel
screen -S control_panel -X stuff "cd ~/arwain_inference_core; python3 python_utils/controller.py\n"
screen -dm -S plot_dashboard
screen -S plot_dashboard -X stuff "cd ~/arwain_inference_core; python3 python_utils/dashboard.py\n"
