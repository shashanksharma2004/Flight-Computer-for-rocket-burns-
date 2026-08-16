# Flight-Computer-for-rocket-burns-
This is just a flight computer which calculates the parameters for the Rocket Flight path done by me at IIMT UNIVERSITY.
Advanced 2D Rocket Flight Simulator
This MATLAB script is a high-fidelity, interactive 2D rocket flight simulator. It models both the real-world physics of a rocket launch (kinematics, aerodynamics, wind drift) and the software logic of an onboard flight computer (sensor fusion, apogee detection, parachute deployment).

🚀 Key Features
Interactive Mission Control: Prompts the user before launch to configure wind speed, rocket mass, launch angle, and parachute deployment altitude.

2D Kinematics & Wind Drift: Calculates both vertical altitude and horizontal downrange drift caused by wind and launch rod angle.

Dynamic Physics: accurately models rocket mass decreasing as motor fuel burns, and air density dropping as altitude increases (Standard Atmosphere Model).

Advanced Avionics (Sensor Fusion): Simulates noisy barometer and accelerometer data, then merges them using a complementary filter to estimate the rocket's true altitude and velocity.

Mach Lockout: Protects the apogee detection logic by intentionally ignoring barometric pressure spikes during the high-speed boost phase.

Dual-Deployment Recovery: Automatically fires a small drogue parachute at apogee, and deploys a larger main parachute when the rocket falls to a user-defined altitude.

Live & Post-Flight Visualization: Draws the rocket's trajectory in real-time during the simulation, and generates a comprehensive data dashboard (Altitude, Velocity, Acceleration) upon touchdown.

🧠 How the Code Works
The simulator is built around a 20 Hz continuous while loop, divided into distinct modules:

1. The Physics Engine (The "Real World")
This section calculates what is actually happening to the rocket. It reads the motor thrust curve, calculates the current air density and drag profile (which changes based on parachute status), and applies 2D Newtonian physics to determine the exact X and Y coordinates of the rocket.

2. The Flight Computer (The "Software")
This section cannot see the "Real World" variables. It relies entirely on simulated sensors (with added random Gaussian noise).

It predicts the rocket's state using the accelerometer (dead reckoning).

It corrects that prediction using the barometer (sensor fusion).

It tracks the flight phase (BOOST, COAST, DESCENT) and watches for three consecutive drops in estimated altitude to confidently trigger the parachutes.

3. The Telemetry System
During the loop, the script limits the plot update rate (using mod() and drawnow limitrate) so the live animation runs smoothly without bogging down the math. Once the physics engine detects Y = 0 (touchdown), the loop breaks and feeds the logged data arrays into a final post-flight dashboard.
