import numpy as np
from python_vehicle_simulator.vehicles import otter
from python_vehicle_simulator.lib import *
from python_vehicle_simulator.lib.gnc import ssa
from plotly.subplots import make_subplots

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go

### PARAMETERS ###
SAMPLE_TIME = 0.02
N = int(60/SAMPLE_TIME)
##################

vehicle = otter(
    controlSystem='headingAutopilotPID',
    r=60.0,
    V_current=0.0,
    beta_current=0.0,
    tau_X=200.0
    ) 


[simTime, simData] = simulate(N, SAMPLE_TIME, vehicle)

### Utils ###
def simulate_vessel(vehicle):
    [simTime, simData] = simulate(N, SAMPLE_TIME, vehicle)
    return simTime, simData

def R2D(value):  # radians to degrees
    return value * 180 / np.pi

## Plotly/ Dash app
app = dash.Dash(__name__)

app.layout = html.Div([
    # Display the image from a URL
    html.Img(src="https://www.researchgate.net/profile/Emanuele-Ferrandino/publication/355760532/figure/fig4/AS:1120829819633666@1644238061866/Motion-of-the-boat-in-6-degree-of-freedom-Fossen-2021.png", style={"width": "400px", "height": "auto"}),

    html.H1("Heading Autopilot"),
    # pilot inputs
    html.H2("Pilot inputs"),
    html.Div([
        html.Label("Surge force (N): "),
        dcc.Input(id="input-tauX", type="number", value=200),
    ], style={"padding": "10px"}),
    
    # Input fields for parameters
    html.H2("PID-controller parameters"),
    html.Div([
        html.Label("P: "),
        dcc.Input(id="input-P", type="number", value=200),
    ], style={"padding": "10px"}),
    
    html.Div([
        html.Label("I: "),
        dcc.Input(id="input-I", type="number", value=0),
    ], style={"padding": "10px"}),
    
    html.Div([
        html.Label("D: "),
        dcc.Input(id="input-D", type="number", value=200),
    ], style={"padding": "10px"}),
    
    html.Div([
        html.Label("Yaw angle setpoint (deg): "),
        dcc.Input(id="input-sp", type="number", value=60),
    ], style={"padding": "10px"}),

    #---
    html.H2("Disturbances"),
    html.Div([
        html.Label("Current speed (m/s): "),
        dcc.Input(id="input-c-speed", type="number", value=0),
    ], style={"padding": "10px"}),

    html.Div([
        html.Label("Current direction (deg): "),
        dcc.Input(id="input-c-dir", type="number", value=0),
    ], style={"padding": "10px"}),
    
    # Graphs to display the curve
    dcc.Graph(id="fig-yaw"),
    dcc.Graph(id="fig-xy"),
])

@app.callback(
    [Output("fig-yaw", "figure"), Output("fig-xy", "figure")],
    #[Input("submit-button", "n_clicks")],
    [Input("input-P", "value"),
     Input("input-I", "value"),
     Input("input-D", "value"),
     Input("input-sp", "value"),
     Input("input-c-speed", "value"),
     Input("input-c-dir", "value"),
     Input("input-tauX", "value")]
)
def update_graph(P, I, D, setpoint, c_speed, c_dir, tauX):
    vehicle.Kp = P
    vehicle.Ki = I
    vehicle.Kd = D
    vehicle.ref = int(((setpoint + 180) % 360) - 180) # values between -180 and 180

    # pilot input surge force
    vehicle.tauX = tauX

    # disturbances
    vehicle.beta_c = c_dir # current speed (m/s)
    vehicle.V_c = c_speed # current direction (deg)

    print(vehicle.ref)

    # Generate x and y data based on inputs
    [time, data] = simulate_vessel(vehicle)
    t = time[:,0]
    x = data[:, 0]
    y = data[:, 1]
    psi = R2D(ssa(data[:, 5]))

    fig_yaw = go.Figure(data=go.Scatter(x=t, y=psi, mode="lines"))
    fig_yaw.update_layout(
        title="Yaw angle",
        xaxis_title="Time (s)",
        yaxis_title="Yaw angle (deg)",
        width=1200,
        height=600
        )

    fig_xy = go.Figure(data=go.Scatter(x=y, y=x, mode="lines"))
    fig_xy.update_layout(
    title="North-East position (m)",
    xaxis_title="x (m)",
    yaxis_title="y (m)",
    xaxis=dict(
        scaleanchor="y", 
        scaleratio=1,
        range=[-200, 200]  # Set x-axis range
    ),
    yaxis=dict(
        scaleanchor="x", 
        scaleratio=1,
        range=[-200, 200]  # Set y-axis range
    ),
    width=800,
    height=800
)

    # Update layout
    #fig.update_layout(height=600, title="Sine and Cosine Curves with Column Spanning", showlegend=False)
    #fig.update_xaxes(title_text="X", row=1, col=1)
    #fig.update_yaxes(title_text="Y", row=1, col=1)
    #fig.update_xaxes(title_text="X", row=2, col=2)
    #fig.update_yaxes(title_text="Y", row=2, col=2)

    return fig_yaw, fig_xy

app.run_server(debug=True)