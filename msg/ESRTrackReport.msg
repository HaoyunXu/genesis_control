Header header

# STATUS/MODE RELATED
int32[] grouping_changed      # changed (1), unchanged(0)
int32[] oncoming              # oncoming (1), not oncoming (0)
int32[] bridge_object         # bridge (1), not bridge (0)
int32[] status                # New_Coasted_Target(7), Invalid_Coasted_Target(6), Merged_Target(5), Coasted_Target(4), 
                            # Updated_Target(3), New_Updated_Target(2), New_Target(1), No_Target(0)
int32[] rolling_count         # ?
int32[] mode                  # both MR/LR (3), LR(2), MR(1), Neither (0)

# MEASUREMENTS
float32[] range               # m
float32[] range_rate          # m/s
float32[] range_accel         # m/s^2
float32[] width               # m
float32[] angle               # deg
float32[] lat_rate            # ?
