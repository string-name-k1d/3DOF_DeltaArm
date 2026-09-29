# State of the arm's terminal endpoint module, published by the arm_endpoint
# node on topic "arm/endpoint/state". Describes whether an end-effector tool /
# payload is attached, and the physical parameters the module contributes to
# dynamics (centre of gravity, mass) used by the CoG / torque estimators.
std_msgs/Header header

# Name of the active endpoint module (e.g. "null", "gripper", "camera_rig").
string module_type

# Dummy connector flag for now: true once a payload/tool is "attached".
# Real hardware will drive this from a connector limit switch / sensor.
bool connector_connected

# True when the attached module is a real (non-stub) implementation.
bool active

# Tool/payload mass in kg (0 for the null module).
float32 payload_mass_kg

# Tool/payload centre of gravity in the endpoint frame, meters.
geometry_msgs/Point payload_cog

# Introspection string for debugging (free-form).
string status
