#!/bin/bash

mkdir /tmp/u200
mkdir /tmp/sailboat

sshfs root@10.42.0.32:/tmp/u200 /tmp/u200/
sshfs root@10.42.0.32:/tmp/sailboat /tmp/sailboat/
