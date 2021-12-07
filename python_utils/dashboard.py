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


all_data = dict()
WD = "/home/pi/arwain_inference_core"


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
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_game_rv.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_game_rv"] = data
        except: pass
        # try:
        #     data = pd.read_csv(f"{WD}/{dataset}/mag_euler_orientation.txt", delimiter=" ").iloc[::10]
        #     data["time"] = (data["time"] - data["time"][0])/1e9
        #     data_load["mag_euler_orientation"] = data
        # except: pass
        try:
            data = pd.read_csv(f"{WD}/{dataset}/madgwick_euler_orientation.txt", delimiter=" ").iloc[::10]
            data["time"] = (data["time"] - data["time"][0])/1e9
            data_load["madgwick_euler_orientation"] = data
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
                    style={"float":"left", "width":"20%", "margin":"5px"}
                ),
                html.Button("Refresh", id="refresh_button", n_clicks=0, style={"float":"left", "margin":"5px"}),
            ],
            style={"display":"flex", "width":"100%", "align-items":"center", "justify-content":"left"}
        )
    ),
    html.Div(
        children=[ 
            html.Div([
                    daq.NumericInput(id='drift_slider',min=-50, max=50, value=0),
                    dcc.Graph(id='position_scatter', figure=fig, style={"height":"90vh"})
                ],
                style={"width":"49%", "float":"left"}
            ),
            html.Div([
                
            ]),
            html.Div(
                [dcc.Graph(id='quaternion_orientation_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
            html.Div(
                [dcc.Graph(id='euler_plot', figure=fig)],
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



## Update dataset list from button #############################################
@app.callback(
    dash.dependencies.Output("dataset_list", "options"),
    dash.dependencies.Input("refresh_button", "n_clicks")
)
def update_list(clicks):
    datasets, _ = get_datasets()
    return [{"label":i,"value":i} for i in datasets]


## Orientation plot callback ###################################################
@app.callback(
    dash.dependencies.Output("quaternion_orientation_plot", "figure"),
    dash.dependencies.Input("dataset_list", "value")
)
def update_orientation_plot(dataset):
    df = read_dataset(dataset, "madgwick_game_rv")
    # fig = px.scatter(df, x=df["x"], y=df["y"], title="Orientation over time")
    fig = go.Figure()
    fig.update_layout(title="Rotation quaternion", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"], y=df["w"], mode="lines", name="w"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["x"], mode="lines", name="x"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["y"], mode="lines", name="y"))
    fig.add_trace(go.Scatter(x=df["time"], y=df["z"], mode="lines", name="z"))
    return fig

## Euler orientation callback ##################################################
@app.callback(
    dash.dependencies.Output("euler_plot", "figure"),
    dash.dependencies.Input("dataset_list","value")
)
def update_euler_plot(dataset):
    df_madgwick = read_dataset(dataset, "madgwick_euler_orientation")
    # df_magn = read_dataset(dataset, "mag_euler_orientation")
    df_diff = read_dataset(dataset, "ori_diff")
    df_madgwick_mag = read_dataset(dataset, "madgwick_mag_euler_orientation")

    fig = go.Figure()
    fig.update_layout(title="Euler yaw [deg]", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    # fig.add_trace(go.Scatter(x=df_madgwick["time"], y=np.unwrap(df_madgwick["roll"], discont=np.pi)*180/np.pi, mode="lines", name="roll"))
    # fig.add_trace(go.Scatter(x=df_madgwick["time"], y=np.unwrap(df_madgwick["pitch"], discont=np.pi)*180/np.pi, mode="lines", name="pitch"))
    fig.add_trace(go.Scatter(x=df_madgwick["time"], y=np.unwrap(df_madgwick["pitch"], discont=np.pi)*180/np.pi, mode="lines", name="Madgwick pitch"))
    fig.add_trace(go.Scatter(x=df_madgwick["time"], y=np.unwrap(df_madgwick["roll"], discont=np.pi)*180/np.pi, mode="lines", name="Madgwick Roll"))
    fig.add_trace(go.Scatter(x=df_madgwick["time"], y=np.unwrap(df_madgwick["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="Madgwick Yaw"))
    # fig.add_trace(go.Scatter(x=df_magn["time"], y=np.unwrap(df_magn["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="Mag yaw"))
    # fig.add_trace(go.Scatter(x=df_diff["time"], y=np.unwrap(df_diff["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="Yaw diff"))
    fig.add_trace(go.Scatter(x=df_madgwick_mag["time"], y=np.unwrap(df_madgwick_mag["pitch"], discont=np.pi)*180/np.pi, mode="lines", name="madg-mag-pitch"))
    fig.add_trace(go.Scatter(x=df_madgwick_mag["time"], y=np.unwrap(df_madgwick_mag["roll"], discont=np.pi)*180/np.pi, mode="lines", name="madg-mag-roll"))
    fig.add_trace(go.Scatter(x=df_madgwick_mag["time"], y=np.unwrap(df_madgwick_mag["yaw"], discont=np.pi)*180/np.pi, mode="lines", name="madg-mag-yaw"))

    return fig



# @app.callback(
#     dash.dependencies.Output("mag_ori_plot", "figure"),
#     dash.dependencies.Input("dataset_list", "value")
# )
# def update_mag_ori_plot(dataset):
    # df = read_dataset(dataset, "mag_euler_orientation")
    # fig = go.Figure()
    # fig.update_layout(title="Magnetic orientation, yaw", title_x=0.5)
    # fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    # fig.add_trace(go.Scatter(x=df["time"], y=df["yaw"], mode="lines", name="mag yaw"))
    # return fig




## Position scatter callback ###################################################
@app.callback(
    dash.dependencies.Output("position_scatter", "figure"),
    [
        dash.dependencies.Input("dataset_list", "value"),
        dash.dependencies.Input("drift_slider", "value")
    ]
)
def update_position_scatter(dataset, drift):
    drift = drift / 60.0 / 20.0
    df = read_dataset(dataset, "position")
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=df["x"], y=df["y"], mode="lines", name="ARWAIN path"))
    fig.update_layout(title="Position", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})

    ## Convert to 2D array, apply drift, convert back to dataframe.
    points_x = list(df["x"])
    points_y = list(df["y"])
    points = [ [points_x[i], points_y[i]] for i in range(1, len(points_x))]
    points = drift_simulator.simulate_drift(points, drift)
    df_drifted = pd.DataFrame(points)

    ## Add the drifted path to the figure.
    fig.add_trace(go.Scatter(x=df_drifted[0], y=df_drifted[1], mode="markers+lines", name="Drifted path"))
    fig.update_layout(title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.update_yaxes(scaleanchor="x", scaleratio=1) # Correct aspect ratio.

    return fig


if __name__ == '__main__':
    app.run_server(host="0.0.0.0", debug=False)
