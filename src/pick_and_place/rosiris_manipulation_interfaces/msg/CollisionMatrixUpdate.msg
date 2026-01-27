string target_link
# whether collisions with the corresponding other_link are allowed
# other_links and collision_allowed must be of same length
CollisionEntry[] collision_entries
# CollisionMatrixUpdate.msg
uint8 mode  # REPLACE, MERGE, or REMOVE
uint8 REPLACE = 0  # Replace all existing ACM entries for target_link
uint8 MERGE = 1    # Update only specified entries, keep others
uint8 REMOVE = 2   # Remove specified entries