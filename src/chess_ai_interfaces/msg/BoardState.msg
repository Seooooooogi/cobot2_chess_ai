# Sparse 8x8 chess board state published by the vision pipeline.
#
# squares[i] is the algebraic square name (e.g. "A1", "E4"); pieces[i] is the
# detected piece code at that square (e.g. "WP" white pawn, "BN" black knight).
# Both arrays must have the same length. Empty squares are omitted.
#
# header.frame_id: "chess_board" (project-defined frame; not REP-105 covered).
# header.stamp:    measurement time (frame capture), not publish time.

std_msgs/Header header

string[] squares
string[] pieces

# DERIVED FIELD: always equals len(squares) == len(pieces).
# Kept for convenience (e.g. roslibjs / web UI display) and to mirror the
# existing Firebase ``chess/board_state.piece_count`` schema. Subscribers
# concerned with correctness should rely on len(squares).
uint8 piece_count
