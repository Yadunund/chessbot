# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""A minimal chess position: FEN in, moves applied, FEN out.

This tracks *where pieces are* and handles the physical side effects of special
moves (castling moves the rook, en passant removes a pawn from another square,
promotion replaces the pawn). It does **not** check legality yet: that arrives
with the rules engine. python-chess is not used because it is GPL-3.0.
"""

from __future__ import annotations

from dataclasses import dataclass

START_FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
FILES = "abcdefgh"


def square_index(name: str) -> int:
    """'a1' -> 0, 'h8' -> 63 (index = file + 8 * rank)."""
    if len(name) != 2 or name[0] not in FILES or name[1] not in "12345678":
        raise ValueError(f"not a square: {name!r}")
    return FILES.index(name[0]) + 8 * (int(name[1]) - 1)


def square_name(index: int) -> str:
    return f"{FILES[index % 8]}{index // 8 + 1}"


def piece_colour(piece: str) -> str:
    return "white" if piece.isupper() else "black"


@dataclass
class MoveEffects:
    """Physical consequences of a move, in the order the arm must carry them out."""

    uci: str
    captured: str | None  # piece symbol removed from the board, if any
    capture_square: str | None  # where it was (differs from destination for en passant)
    rook_from: str | None  # castling rook
    rook_to: str | None
    promotion: str | None  # piece symbol the pawn becomes


class Board:
    def __init__(self, fen: str = START_FEN):
        placement, side, castling, en_passant, halfmove, fullmove = (fen.split() + ["w", "-", "-", "0", "1"])[:6]
        self.squares: list[str | None] = [None] * 64
        rank = 7
        for row in placement.split("/"):
            file = 0
            for ch in row:
                if ch.isdigit():
                    file += int(ch)
                else:
                    self.squares[file + 8 * rank] = ch
                    file += 1
            rank -= 1
        self.white_to_move = side == "w"
        self.castling = castling
        self.en_passant = en_passant
        self.halfmove = int(halfmove)
        self.fullmove = int(fullmove)

    def fen(self) -> str:
        rows = []
        for rank in range(7, -1, -1):
            row, empty = "", 0
            for file in range(8):
                piece = self.squares[file + 8 * rank]
                if piece is None:
                    empty += 1
                else:
                    row += (str(empty) if empty else "") + piece
                    empty = 0
            rows.append(row + (str(empty) if empty else ""))
        side = "w" if self.white_to_move else "b"
        return f"{'/'.join(rows)} {side} {self.castling or '-'} {self.en_passant} {self.halfmove} {self.fullmove}"

    def piece_at(self, square: str) -> str | None:
        return self.squares[square_index(square)]

    def effects(self, uci: str) -> MoveEffects:
        """Work out what physically happens for a move, without applying it."""
        src, dst = uci[:2], uci[2:4]
        piece = self.piece_at(src)
        if piece is None:
            raise ValueError(f"no piece on {src}")
        captured = self.piece_at(dst)
        capture_square = dst if captured else None
        rook_from = rook_to = None
        if piece in "Kk" and abs(square_index(src) % 8 - square_index(dst) % 8) == 2:
            rank = src[1]
            if dst[0] == "g":
                rook_from, rook_to = f"h{rank}", f"f{rank}"
            else:
                rook_from, rook_to = f"a{rank}", f"d{rank}"
        if piece in "Pp" and dst == self.en_passant and captured is None:
            capture_square = f"{dst[0]}{src[1]}"
            captured = self.piece_at(capture_square)
        promotion = None
        if piece in "Pp" and dst[1] in "18":
            symbol = uci[4] if len(uci) > 4 else "q"
            promotion = symbol.upper() if piece.isupper() else symbol.lower()
        return MoveEffects(uci, captured, capture_square, rook_from, rook_to, promotion)

    def apply(self, uci: str) -> MoveEffects:
        effects = self.effects(uci)
        src, dst = square_index(uci[:2]), square_index(uci[2:4])
        piece = self.squares[src]
        if effects.capture_square:
            self.squares[square_index(effects.capture_square)] = None
        self.squares[dst] = effects.promotion or piece
        self.squares[src] = None
        if effects.rook_from and effects.rook_to:
            self.squares[square_index(effects.rook_to)] = self.squares[square_index(effects.rook_from)]
            self.squares[square_index(effects.rook_from)] = None

        # Bookkeeping that later moves depend on.
        if piece in "Pp" and abs(dst - src) == 16:
            self.en_passant = square_name((src + dst) // 2)
        else:
            self.en_passant = "-"
        if piece in "Kk":
            self.castling = self.castling.replace("K" if piece == "K" else "k", "").replace(
                "Q" if piece == "K" else "q", ""
            )
        self.halfmove = 0 if piece in "Pp" or effects.captured else self.halfmove + 1
        if not self.white_to_move:
            self.fullmove += 1
        self.white_to_move = not self.white_to_move
        return effects
