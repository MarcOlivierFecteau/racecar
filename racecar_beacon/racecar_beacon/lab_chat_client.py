#!/usr/bin/env python3
import socket

HOST = '127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))