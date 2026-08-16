# LogNode가 SQLite commit을 완료한 뒤에만 발행합니다.
CommonHeader header
string producer_node
string producer_instance_id
string[] acked_log_ids
uint32[] acked_revisions
builtin_interfaces/Time committed_at
