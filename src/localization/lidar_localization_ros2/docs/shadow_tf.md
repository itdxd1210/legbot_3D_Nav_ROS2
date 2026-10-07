# Shadow comparison TF output

`publish_tf` defaults to `true`, preserving normal localization behavior.
Set it to `false` for shadow comparisons: pose and diagnostic topics remain
available, but all localization TF broadcasts are suppressed. Input TF lookup
continues to use the normal robot transform tree.

`enable_map_odom_tf` only selects the output transform type; setting that option
to false alone still publishes map-to-base and is not a shadow-mode switch.
