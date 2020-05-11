#!/bin/bash

set -exo pipefail

dir=~
if [[ -n ${1} ]]; then
      dir=${1}
fi

# NO OP
