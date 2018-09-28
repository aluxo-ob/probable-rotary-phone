Header header

# bitfield indicating status
# negative values (most significant bit set) indicate an error condition
# positive values are for other status information
int64 status

# Error information; blank if everything is fine
string status_message

# Tracked objects and their properties
TrackedObject[] objects
