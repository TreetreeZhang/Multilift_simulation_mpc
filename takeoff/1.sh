#!/usr/bin/env bash
set -eo pipefail

conda deactivate || true
MicroXRCEAgent udp4 -p 8888
