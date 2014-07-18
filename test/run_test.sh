#!/bin/sh

../bin/scan3d-capture &

python fake_sync_broadcast.py
