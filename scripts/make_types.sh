#!/bin/bash

set -euo pipefail


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting LCM type generation...${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LCM_TYPES_DIR="${SCRIPT_DIR}/../lcm-types"

if ! command -v lcm-gen >/dev/null 2>&1; then
  echo "ERROR: lcm-gen was not found. Install LCM development tools first." >&2
  echo "       On Ubuntu this is usually: sudo apt-get install liblcm-dev lcm" >&2
  exit 1
fi

if [ ! -d "${LCM_TYPES_DIR}" ]; then
  echo "ERROR: lcm-types directory not found: ${LCM_TYPES_DIR}" >&2
  exit 1
fi

cd "${LCM_TYPES_DIR}"
# Clean
find . -mindepth 2 -maxdepth 2 \( -name '*.jar' -o -name '*.java' -o -name '*.hpp' -o -name '*.class' -o -name '*.py' -o -name '*.pyc' \) -delete

# Make
lcm-gen -jxp *.lcm
if [ -f /usr/local/share/java/lcm.jar ]; then
  cp /usr/local/share/java/lcm.jar .
elif [ -f /usr/share/java/lcm.jar ]; then
  cp /usr/share/java/lcm.jar .
else
  echo "ERROR: lcm.jar was not found under /usr/local/share/java or /usr/share/java." >&2
  exit 1
fi
javac -cp lcm.jar */*.java
jar cf my_types.jar */*.class
mkdir -p java
mv my_types.jar java
mv lcm.jar java
mkdir -p cpp
mv *.hpp cpp

mkdir -p python
mv *.py python

FILES=$(ls */*.class)
echo ${FILES} > file_list.txt


echo -e "${GREEN} Done with LCM type generation${NC}"
