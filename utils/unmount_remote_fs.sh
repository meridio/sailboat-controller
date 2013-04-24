#!/bin/bash

fusermount -u /tmp/u200
fusermount -u /tmp/sailboat

sudo umount -l /tmp/u200
sudo umount -l /tmp/sailboat

rmdir /tmp/u200
rmdir /tmp/sailboat
