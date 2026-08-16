# 멱등 명령 식별자. command_id는 UUIDv4, payload_digest는 SHA-256 hex입니다.
CommonHeader header
uint64 command_epoch
string command_id
string payload_digest
builtin_interfaces/Time issued_at
