# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import os
import dash
from dash import dcc
from dash import html
import dash_bootstrap_components as dbc
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
import drift_simulator


WD = "/home/pi/arwain_inference_core"


def get_datasets():
    datasets = sorted([folder for folder in  os.listdir(WD) if folder.startswith("data")])
    if len(datasets) == 0:
        return ["No data available"], None
    df = pd.read_csv(f"{WD}/{datasets[0]}/position.txt", delimiter=" ")
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
                        id="dataset-list",
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
                    dcc.Slider(id='drift_slider',min=-5,max=5,step=0.05,value=1),
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
            html.Div(
                [dcc.Graph(id='mag_ori_plot', figure=fig)],
                style={"width":"49%","float":"left"}
            ),
        ],
        style={"margin":"auto","width":"100%","display":"inline-block"})
    ],
    style={"font-family":"sans-serif"}
)


@app.callback(
    dash.dependencies.Output("gyro_plot", "figure"),
    dash.dependencies.Input("dataset-list", "value")
)
def update_gyro_plot(dataset):
    path = f"{WD}/{dataset}/world_gyro.txt"
    df = pd.read_csv(path, delimiter=" ")
    fig = go.Figure()
    fig.update_layout(title="Angular velocity, world frame [rad/s]", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["x"], mode="lines", name="x"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["y"], mode="lines", name="y"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["z"], mode="lines", name="z"))
    return fig

@app.callback(
    dash.dependencies.Output("accel_plot", "figure"),
    dash.dependencies.Input("dataset-list", "value")
)
def update_accel_plot(dataset):
    path = f"{WD}/{dataset}/world_acce.txt"
    df = pd.read_csv(path, delimiter=" ")
    fig = go.Figure()
    fig.update_layout(title="Linear acceleration, world frame [m/s2]", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["x"], mode="lines", name="x"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["y"], mode="lines", name="y"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["z"], mode="lines", name="z"))
    return fig


@app.callback(
    dash.dependencies.Output("mag_ori_plot", "figure"),
    dash.dependencies.Input("dataset-list", "value")
)
def update_mag_ori_plot(dataset):
    path = f"{WD}/{dataset}/mag_orientation.txt"
    df = pd.read_csv(path, delimiter=" ")
    fig = go.Figure()
    fig.update_layout(title="Magnetic orientation, quaternion", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["w"], mode="lines", name="w"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["x"], mode="lines", name="x"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["y"], mode="lines", name="y"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["z"], mode="lines", name="z"))
    return fig



## Update dataset list from button #############################################
@app.callback(
    dash.dependencies.Output("dataset-list", "options"),
    dash.dependencies.Input("refresh_button", "n_clicks")
)
def update_list(clicks):
    datasets, _ = get_datasets()
    return [{"label":i,"value":i} for i in datasets]


## Orientation plot callback ###################################################
@app.callback(
    dash.dependencies.Output("quaternion_orientation_plot", "figure"),
    dash.dependencies.Input("dataset-list", "value")
)
def update_orientation_plot(dataset):
    path = f"{WD}/{dataset}/game_rv.txt"
    df = pd.read_csv(path, delimiter=" ")
    # fig = px.scatter(df, x=df["x"], y=df["y"], title="Orientation over time")
    fig = go.Figure()
    fig.update_layout(title="Rotation quaternion", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["w"], mode="lines", name="w"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["x"], mode="lines", name="x"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["y"], mode="lines", name="y"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["z"], mode="lines", name="z"))
    return fig

## Euler orientation callback ##################################################
@app.callback(
    dash.dependencies.Output("euler_plot", "figure"),
    dash.dependencies.Input("dataset-list","value")
)
def update_euler_plot(dataset):
    path = f"{WD}/{dataset}/euler_orientation.txt"
    df = pd.read_csv(path, delimiter=" ")
    fig = go.Figure()
    fig.update_layout(title="Euler orientation [rad]", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["roll"], mode="lines", name="roll"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["pitch"], mode="lines", name="pitch"))
    fig.add_trace(go.Scatter(x=df["time"]/1e17, y=df["yaw"], mode="lines", name="yaw"))
    return fig

## Position scatter callback ###################################################
@app.callback(
    dash.dependencies.Output("position_scatter", "figure"),
    [
        dash.dependencies.Input("dataset-list", "value"),
        dash.dependencies.Input("drift_slider", "value")
    ]
)
def update_position_scatter(dataset, drift):
    path = f"{WD}/{dataset}/position.txt"
    df = pd.read_csv(path, delimiter=" ")
    num_rows = 200
    df = df.head(num_rows)
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=df["x"], y=df["y"], mode="lines", name="ARWAIN path"))
    fig.update_layout(title="Position", title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})

    ## Convert to 2D array, apply drift, convert back to dataframe.
    points_x = list(df["x"])
    points_y = list(df["y"])
    points = [ [points_x[i], points_y[i]] for i in range(len(points_x))]
    points = drift_simulator.simulate_drift(points, drift)
    df = pd.DataFrame(points)
    df = df.head(num_rows)

    ## Add the drifted path to the figure.
    fig.add_trace(go.Scatter(x=df[0], y=df[1], mode="lines", name="Drifted path"))
    fig.update_layout(title_x=0.5)
    fig.update_layout(margin={"l":40, "r":40, "t":40, "b":40})
    fig.update_yaxes(scaleanchor="x", scaleratio=1) # Correct aspect ratio.

    return fig


if __name__ == '__main__':
    app.run_server(host="0.0.0.0", debug=True)
