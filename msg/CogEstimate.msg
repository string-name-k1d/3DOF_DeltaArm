# Estimated combined centre of gravity of the arm + attached payload, published
# by the arm_endpoint node on topic "arm/endpoint/cog_estimate".
#
# Skeleton message: values come from the endpoint module (payload CoG) combined
# with the arm's own CoG. A future estimator can fill these from real dynamics;
# the fields are already shaped for that.
std_msgs/Header header

# Estimated combined CoG in the base frame, meters.
geometry_msgs/Point cog_base

# Estimated combined CoG in the endpoint frame, meters.
geometry_msgs/Point cog_endpoint

# Combined mass estimate in kg (arm + payload).
float32 total_mass_kg

# Confidence in [0, 1]; 1.0 = exact (stub returns 1.0).
float32 confidence
