#!/bin/bash

# Path to Script
cd -- "$(dirname "$(find ~/ -type f -name "cobot.sh" | grep "bash/cobot.sh")")"

# Path to Repo
cd ..

# Override Project Path
echo -e "#!/bin/bash\n\n# Path to Project\nREPO_PATH=\"$PWD\"" > bash/path.sh
echo -e "\n# Source Path to Variables\nsource \$REPO_PATH/bash/variables.sh" >> bash/path.sh
echo -e "\n# Source Path to Cobot\nsource \$REPO_PATH/bash/cobot.sh" >> bash/path.sh
echo -e "\n# Source Path to AI\nsource \$REPO_PATH/bash/ai.sh" >> bash/path.sh

# Append Source Path to bashrc
echo -e "\nsource $PWD/bash/path.sh" >> ~/.bashrc
