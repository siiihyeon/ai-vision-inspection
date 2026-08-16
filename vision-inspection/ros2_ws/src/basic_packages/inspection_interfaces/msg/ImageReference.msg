# VisionNode가 소유하는 canonical RGB PNG에 대한 불변 참조입니다.
string camera_id
string file_path
string sha256
uint64 file_size_bytes
uint32 width
uint32 height
string pixel_format
# camera_timestamp_raw/domain은 PTP 동기화 가능 여부와 무관하게 보존합니다.
uint64 camera_timestamp_raw
string camera_timestamp_domain
builtin_interfaces/Time camera_timestamp
uint64 host_arrival_monotonic_ns
builtin_interfaces/Time host_arrival_timestamp
