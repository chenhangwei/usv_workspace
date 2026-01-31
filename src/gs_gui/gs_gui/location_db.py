#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple SQLite helper for saving/loading named location offsets.
"""
import os
import sqlite3
from typing import Optional, List, Dict


def _get_db_path() -> str:
    # store DB in user home to avoid permission issues
    home = os.path.expanduser("~")
    db_path = os.path.join(home, ".usv_locations.db")
    return db_path


def _connect():
    path = _get_db_path()
    conn = sqlite3.connect(path, timeout=5)
    return conn


def init_db() -> None:
    conn = _connect()
    try:
        cur = conn.cursor()
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS locations (
                name TEXT PRIMARY KEY,
                x REAL,
                y REAL,
                z REAL,
                angle INTEGER
            )
            """
        )
        conn.commit()
    finally:
        conn.close()


def get_all_names() -> List[str]:
    init_db()
    conn = _connect()
    try:
        cur = conn.cursor()
        cur.execute("SELECT name FROM locations ORDER BY name COLLATE NOCASE")
        rows = cur.fetchall()
        return [r[0] for r in rows]
    finally:
        conn.close()


def get_location_by_name(name: str) -> Optional[Dict]:
    if not name:
        return None
    init_db()
    conn = _connect()
    try:
        cur = conn.cursor()
        cur.execute("SELECT x,y,z,angle FROM locations WHERE name = ?", (name,))
        row = cur.fetchone()
        if row is None:
            return None
        return {'x': float(row[0]), 'y': float(row[1]), 'z': float(row[2]), 'angle': int(row[3])}
    finally:
        conn.close()


def upsert_location(name: str, x: float, y: float, z: float, angle: int) -> None:
    if not name:
        raise ValueError("name required")
    init_db()
    conn = _connect()
    try:
        cur = conn.cursor()
        cur.execute(
            "REPLACE INTO locations (name,x,y,z,angle) VALUES (?,?,?,?,?)",
            (name, float(x), float(y), float(z), int(angle)),
        )
        conn.commit()
    finally:
        conn.close()


def delete_location(name: str) -> None:
    if not name:
        return
    init_db()
    conn = _connect()
    try:
        cur = conn.cursor()
        cur.execute("DELETE FROM locations WHERE name = ?", (name,))
        conn.commit()
    finally:
        conn.close()
