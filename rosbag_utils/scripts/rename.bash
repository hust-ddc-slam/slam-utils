#!/bin/bash

# Counter starting at 0
COUNTER=0

# Loop through all files with .bag extension
for FILE in *.bag; do
    echo "$FILE"

    # Skip if the file is a directory
    if [ -d "$FILE" ]; then
        continue
    fi


    # Rename the file
    mv "$FILE" "${COUNTER}.bag"

    # Increment the counter
    ((COUNTER++))
done

echo "Renaming complete."


