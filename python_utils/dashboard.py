# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import os
import dash
from dash import dcc
from dash import html
import dash_daq as daq
import dash_bootstrap_components as dbc
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
import drift_simulator
import numpy as np
import subprocess
import time


all_data = dict()

try:
    WD = sys.argv[1]
    if WD[-1] == "/":
        WD = WD[:-1]
except IndexError:
    WD = "/home/pi/arwain_inference_core"

time_slider = dcc.RangeSlider(
    min=0,
    max=10000//20,
    step=5,
    marks={x:str(x) for x in range(0, 10000//20, 1000//20)},
    value=[0, 10000//20],
    allowCross=True,
    pushable=10,
    disabled=False,
    dots=True,
    included=True,
    updatemode="mouseup",
    vertical=False,
    id="time_slider"
)

def read_dataset(dataset, facet):
    ## If this is a new dataset, load it.
    if all_data.get(dataset) is None:
        data_load = dict()
        try:
            data = pd.read_csv(f"{WD}/{dataset}/position.txt", delimiter=" ")
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["position"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/velocity.txt", delimiter=" ")
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["velocity"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/kalman_position.txt", delimiter=" ")
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["kalman_position"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/uwb_log.txt", delimiter=" ")
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["uwb_position"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/world_acce.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["world_acce"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/world_gyro.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["world_gyro"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_game_rv_1.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_game_rv_1"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_game_rv_2.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_game_rv_2"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_game_rv_3.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_game_rv_3"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_euler_orientation_1.txt", delimiter=" ")#.iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_euler_orientation_1"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_euler_orientation_2.txt", delimiter=" ")#.iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_euler_orientation_2"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_euler_orientation_3.txt", delimiter=" ")#.iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_euler_orientation_3"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/ori_diff.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["ori_diff"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_mag_euler_orientation.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_mag_euler_orientation"] = data
        except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/pressure.txt", delimiter=" ")
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["pressure"] = data
        except: pass
        all_data[dataset] = data_load
    return all_data[dataset].get(facet)


def get_datasets():
    datasets = sorted([folder for folder in  os.listdir(WD) if folder.startswith("data")])
    if len(datasets) == 0:
        return ["No data available"], None
    df = read_dataset(datasets[0], "position")
    return datasets, df


## Get the datasets.
datasets, df = get_datasets()

## Default figure.
fig = px.scatter()

## Create layout.
external_stylesheets = [dbc.themes.DARKLY]

## Create dash app + layout.
app = dash.Dash(__name__, title="ARWAIN Tracking", external_stylesheets=[external_stylesheets])
app.layout = html.Div(children=[
    html.Div(
        html.Div(
            children=[
                html.H1(children='ARWAIN Tracking', style={"float":"left", "margin":"5px"}),
                html.Div(
                    dcc.Dropdown(
                        id="dataset_list",
                        options=[{"label":i,"value":i} for i in datasets],
                        value=datasets[0],
                        clearable=False,
                    ),
                    style={"float":"left", "width":"40%", "margin":"5px"}
                ),
                html.Button("Refresh", id="refresh_button", n_clicks=0, style={"float":"left", "margin":"5px"}),
                html.Button("Delete", id="delete_button", n_clicks=0, style={"float":"right", "margin":"5px"}),
                html.Div(id="hidden-div", style={"display":"none"})
            ],
            style={"display":"flex", "width":"100%", "align-items":"center", "justify-content":"left"}
        )
    ),
    html.Div(
        children=[ 
            html.Div([
                    # daq.NumericInput(id='drift_slider',min=-50, max=50, value=0),
                    dcc.Graph(id='position_scatter', figure=fig, style={"height":"90vh"}),
                    time_slider
                ],
            ),
            html.Div(
                [dcc.Graph(id='velocity_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
            html.Div(
                [dcc.Graph(id='euler_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
            html.Div(
                [dcc.Graph(id='pressure_temperature_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
            html.Div(
                [dcc.Graph(id='pressure_altitude_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
            html.Div(
                [dcc.Graph(id='ai_altitude_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
            html.Div(
                [dcc.Graph(id='accel_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
            html.Div(
                [dcc.Graph(id='gyro_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
        ],
        style={"margin":"auto","width":"100%","display":"inline-block"})
    ],
    style={"font-family":"sans-serif"}
)


@app.callback(
    dash.dependencies.Output("gyro_plot", "figure"),
    dash.dependencies.Input("dataset_list", "value")
)
def update_gyro_plot(dataset):
    df = read_dataset(dataset, "world_gyro")
    fig = go.Figure()
    fig.update_layout(title="Angular velocity, world frame [rad/s]", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"], y=df["x"], mode="lines", name="x"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["y"], mode="lines", name="y"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["z"], mode="lines", name="z"))
    return fig


@app.callback(
    dash.dependencies.Output("accel_plot", "figure"),
    dash.dependencies.Input("dataset_list", "value")
)
def update_accel_plot(dataset):
    df = read_dataset(dataset, "world_acce")
    fig = go.Figure()
    fig.update_layout(title="Linear acceleration, world frame [m/s2]", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"], y=df["x"], mode="lines", name="x"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["y"], mode="lines", name="y"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["z"], mode="lines", name="z"))
    return fig


@app.callback(
    dash.dependencies.Output("velocity_plot", "figure"),
    dash.dependencies.Input("dataset_list", "value")
)
def update_velocity_plot(dataset):
    df = read_dataset(dataset, "velocity")
    fig = go.Figure()
    fig.update_layout(title="World velocity", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"], y=df["x"], mode="lines", name="Velocity x [m/s]"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["y"], mode="lines", name="Velocity y [m/s]"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["z"], mode="lines", name="Velocity z [m/s]"))
    return fig 




## Update dataset list from button #############################################
@app.callback(
    dash.dependencies.Output("hidden-div", "title"),
    dash.dependencies.Input("delete_button", "n_clicks"),
    dash.dependencies.State("dataset_list", "value")
)
def delete_data(clicks, selected_dataset):
    if clicks == 0 or clicks is None:
        return "loaded"
    subprocess.Popen(["rm", "-rf", "/home/pi/arwain_inference_core/" + selected_dataset])
    time.sleep(0.5)
    return "done"


## Update dataset list from button #############################################
@app.callback(
    dash.dependencies.Output("dataset_list", "options"),
    dash.dependencies.Input("refresh_button", "n_clicks")
)
def update_list(clicks):
    datasets, _ = get_datasets()
    return [{"label":i,"value":i} for i in datasets]


## Euler orientation callback ##################################################
@app.callback(
    dash.dependencies.Output("euler_plot", "figure"),
    dash.dependencies.Input("dataset_list","value")
)
def update_euler_plot(dataset):

    fig = go.Figure()
    fig.update_layout(title="Euler yaw [deg]", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    try:
        df_madgwick = read_dataset(dataset, "madgwick_euler_orientation_1")
        fig.add_trace(go.Scatter(x=df_madgwick["time"], y=np.unwrap(df_madgwick["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="Madgwick Yaw 1"))
    except: pass
    try:
        df_madgwick_2 = read_dataset(dataset, "madgwick_euler_orientation_2")
        fig.add_trace(go.Scatter(x=df_madgwick_2["time"], y=np.unwrap(df_madgwick_2["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="Madgwick Yaw 2"))
    except: pass
    try:
        df_madgwick_3 = read_dataset(dataset, "madgwick_euler_orientation_3")
        fig.add_trace(go.Scatter(x=df_madgwick_3["time"], y=np.unwrap(df_madgwick_3["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="Madgwick Yaw 3"))
    except: pass
    try:
        df_diff = read_dataset(dataset, "ori_diff")
        fig.add_trace(go.Scatter(x=df_diff["time"], y=np.unwrap(df_diff["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="Yaw diff"))
    except: pass
    try:
        df_madgwick_mag = read_dataset(dataset, "madgwick_mag_euler_orientation")
        fig.add_trace(go.Scatter(x=df_madgwick_mag["time"], y=np.unwrap(df_madgwick_mag["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="madg-mag-yaw"))
    except: pass

    return fig


@app.callback(
    dash.dependencies.Output("pressure_altitude_plot", "figure"),
    dash.dependencies.Input("dataset_list", "value")
)
def update_altitude_plot(dataset):
    df_altitude = read_dataset(dataset, "pressure")
    fig = go.Figure()
    fig.update_layout(title="Altitude (from pressure/temperature)", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df_altitude["time"], y=df_altitude["altitude"], mode="lines", name="Altitude [m]"))
    return fig


@app.callback(
    dash.dependencies.Output("ai_altitude_plot", "figure"),
    dash.dependencies.Input("dataset_list", "value")
)
def update_altitude_plot(dataset):
    df_altitude = read_dataset(dataset, "position")
    fig = go.Figure()
    fig.update_layout(title="Altitude (from NN)", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df_altitude["time"], y=df_altitude["z"], mode="lines", name="Altitude [m]"))
    return fig


@app.callback(
    dash.dependencies.Output("pressure_temperature_plot", "figure"),
    dash.dependencies.Input("dataset_list","value")
)
def update_pressure_temperature_plot(dataset):
    df_pressure = read_dataset(dataset, "pressure")
    fig = go.Figure()
    fig.update_layout(title="Pressure/Temperature", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df_pressure["time"], y=df_pressure["pressure"]/1e5, mode="lines", name="Pressure [hPa/1000]"))
    fig.add_trace(go.Scatter(x=df_pressure["time"], y=df_pressure["temperature"], mode="lines", name="Temperature [°C]"))
    return fig


@app.callback(
    dash.dependencies.Output("position_scatter", "figure"),
    [
        dash.dependencies.Input("dataset_list", "value"),
        # dash.dependencies.Input("drift_slider", "value"),
        dash.dependencies.Input("time_slider", "value")
    ]
)
def update_position_scatter(dataset, slider_values):
    # drift = drift / 60.0 / 20.0
    # df = read_dataset(dataset, "position")
    
    # fig = go.Figure()
    # fig.add_trace(go.Scatter(x=df["x"], y=df["y"], mode="lines", name="ARWAIN path"))
    
    # ## Add the UWB path if it exists.
    # try:
    #     df2 = read_dataset(dataset, "uwb_log")
    #     fig.add_trace(go.Scatter(x=df2["x"], y=df2["y"], mode="lines", name="UWB path"))
    # except:
    #     pass
    # fig.update_layout(title="Position", title_x=0.5)
    # fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})

    ## Convert to 2D array, apply drift, convert back to dataframe.
    #points_x = list(df["x"])
    #points_y = list(df["y"])
    #points = [ [points_x[i], points_y[i]] for i in range(1, len(points_x))]
    #points, best_drift = drift_simulator.find_best_drift_correction(points)

    # points = drift_simulator.simulate_drift(points, drift)
    #df_drifted = pd.DataFrame(points)

    ## Add the drifted path to the figure.
    #fig.add_trace(go.Scatter(x=df_drifted[0], y=df_drifted[1], mode="lines", name=f"Drifted path by {best_drift}"))
    # fig.update_layout(title_x=0.5)
    # fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    # fig.update_yaxes(scaleanchor="x", scaleratio=1) # Correct aspect ratio.

    # return fig

    
    start = slider_values[0]*20
    end = slider_values[1]*20

    fig = go.Figure()

    df = read_dataset(dataset, "position")
    fig.add_trace(go.Scatter(x=df["x"][start:end], y=df["y"][start:end], text=df["time"][start:end], mode="lines", name="ARWAIN path"))
    
    try:
        df_k = read_dataset(dataset, "kalman_position")
        fig.add_trace(go.Scatter(x=df_k["x"][start:end], y=df_k["y"][start:end], text=df_k["time"][start:end], mode="lines", name="Kalman path"))
    except:
        pass
    try:
        df_u = read_dataset(dataset, "uwb_position")
        fig.add_trace(go.Scatter(x=df_u["x"][start:end], y=df_u["y"][start:end], text=df_u["time"][start:end], mode="lines", name="UWB path"))
    except:
        pass
    
    fig.update_layout(title="Position", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})

    fig.update_layout(title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.update_yaxes(scaleanchor="x", scaleratio=1) # Correct aspect ratio.
    return fig


if __name__ == '__main__':
    app.run_server(host="0.0.0.0", debug=True)
