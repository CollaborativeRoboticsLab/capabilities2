# supported types of option
uint8 BOOL = 0
uint8 DOUBLE = 1
uint8 INT = 2
uint8 STRING = 3
uint8 VECTOR_BOOL = 4
uint8 VECTOR_DOUBLE = 5
uint8 VECTOR_INT = 6
uint8 VECTOR_STRING = 7


# Key of the option
string key

# Value array of option for vector types. 
# insert a single value for non-vector types.
string[] value

# Type of the option
uint8 type