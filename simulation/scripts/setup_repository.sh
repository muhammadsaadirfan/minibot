#!/bin/bash

# Mobile Robot Simulation - Repository Setup Script
# This script helps set up the GitHub repository for the open source project

set -e  # Exit on any error

echo "🤖 Mobile Robot Simulation - Repository Setup"
echo "=============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}[SETUP]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "src/mobile_robot/package.xml" ]; then
    print_error "This script must be run from the root of the mobile robot simulation workspace"
    exit 1
fi

print_header "Starting repository setup..."

# Check if git is installed
if ! command -v git &> /dev/null; then
    print_error "Git is not installed. Please install git first."
    exit 1
fi

# Check if git repository is already initialized
if [ -d ".git" ]; then
    print_warning "Git repository already exists. Do you want to continue? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        print_status "Setup cancelled."
        exit 0
    fi
else
    print_status "Initializing git repository..."
    git init
fi

# Configure git (if not already configured)
print_status "Configuring git..."
if [ -z "$(git config --global user.name)" ]; then
    print_warning "Git user.name not set. Please enter your name:"
    read -r git_name
    git config --global user.name "$git_name"
fi

if [ -z "$(git config --global user.email)" ]; then
    print_warning "Git user.email not set. Please enter your email:"
    read -r git_email
    git config --global user.email "$git_email"
fi

# Add all files to git
print_status "Adding files to git..."
git add .

# Create initial commit
print_status "Creating initial commit..."
git commit -m "Initial commit: Mobile Robot Simulation for ROS Melodic

- Custom 4-wheel differential drive robot (TortoiseBot)
- Complete URDF robot model with caster wheels
- Custom differential drive hardware interface
- Gazebo simulation world with obstacles
- SLAM mapping capabilities using gmapping
- Navigation stack integration with move_base
- RViz visualization configurations
- Pre-built maps and controller configurations
- Comprehensive documentation and tutorials
- Open source setup with MIT license"

# Create main branch (if not exists)
if [ "$(git branch --show-current)" != "main" ]; then
    print_status "Creating main branch..."
    git branch -M main
fi

# Add remote repository
print_status "Setting up remote repository..."
print_warning "Please enter your GitHub repository URL (e.g., https://github.com/username/mobile-robot-simulation.git):"
read -r repo_url

if [ -n "$repo_url" ]; then
    git remote add origin "$repo_url"
    print_status "Remote repository added: $repo_url"
else
    print_warning "No repository URL provided. You can add it later with:"
    echo "git remote add origin <your-repo-url>"
fi

# Create .gitignore if it doesn't exist
if [ ! -f ".gitignore" ]; then
    print_status "Creating .gitignore file..."
    cat > .gitignore << 'EOF'
# ROS Melodic Workspace
# Build artifacts
build/
devel/
install/

# Log files
log/
*.log

# Python cache
__pycache__/
*.py[cod]
*$py.class
*.so

# C++ build artifacts
*.o
*.a
*.so
*.dylib
*.dll

# CMake
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
Makefile
*.cmake

# IDE files
.vscode/
.idea/
*.swp
*.swo
*~

# OS generated files
.DS_Store
.DS_Store?
._*
.Spotlight-V100
.Trashes
ehthumbs.db
Thumbs.db

# Temporary files
*.tmp
*.temp
*.bak
*.backup

# ROS specific
*.bag
*.bag.active
*.bag.bak

# Gazebo
~/.gazebo/

# Catkin
.catkin_workspace

# Environment variables
.env

# Documentation build
docs/_build/

# Test coverage
.coverage
htmlcov/

# Package lock files (if using npm/yarn for any web components)
package-lock.json
yarn.lock

# Backup files
*.orig
*.rej
EOF
    git add .gitignore
    git commit -m "Add .gitignore file for ROS workspace"
fi

# Create GitHub-specific files
print_status "Creating GitHub-specific files..."

# Create issue templates
mkdir -p .github/ISSUE_TEMPLATE

cat > .github/ISSUE_TEMPLATE/bug_report.md << 'EOF'
---
name: Bug report
about: Create a report to help us improve
title: '[BUG] '
labels: bug
assignees: ''

---

**Describe the bug**
A clear and concise description of what the bug is.

**To Reproduce**
Steps to reproduce the behavior:
1. Go to '...'
2. Click on '....'
3. Scroll down to '....'
4. See error

**Expected behavior**
A clear and concise description of what you expected to happen.

**Screenshots**
If applicable, add screenshots to help explain your problem.

**Environment (please complete the following information):**
 - OS: [e.g. Ubuntu 18.04]
 - ROS Version: [e.g. Melodic]
 - Hardware: [e.g. CPU, RAM, GPU]
 - Installation method: [e.g. Source/Binary]

**Additional context**
Add any other context about the problem here.
EOF

cat > .github/ISSUE_TEMPLATE/feature_request.md << 'EOF'
---
name: Feature request
about: Suggest an idea for this project
title: '[FEATURE] '
labels: enhancement
assignees: ''

---

**Is your feature request related to a problem? Please describe.**
A clear and concise description of what the problem is. Ex. I'm always frustrated when [...]

**Describe the solution you'd like**
A clear and concise description of what you want to happen.

**Describe alternatives you've considered**
A clear and concise description of any alternative solutions or features you've considered.

**Additional context**
Add any other context or screenshots about the feature request here.
EOF

# Create pull request template
cat > .github/pull_request_template.md << 'EOF'
## Description
Please include a summary of the change and which issue is fixed. Please also include relevant motivation and context.

Fixes # (issue)

## Type of change
Please delete options that are not relevant.

- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] This change requires a documentation update

## How Has This Been Tested?
Please describe the tests that you ran to verify your changes. Provide instructions so we can reproduce. Please also list any relevant details for your test configuration.

- [ ] Test A
- [ ] Test B

## Checklist:
- [ ] My code follows the style guidelines of this project
- [ ] I have performed a self-review of my own code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to the documentation
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix is effective or that my feature works
- [ ] New and existing unit tests pass locally with my changes
- [ ] Any dependent changes have been merged and published in downstream modules
EOF

git add .github/
git commit -m "Add GitHub issue and pull request templates"

# Final instructions
print_header "Setup completed successfully!"
echo ""
echo "Next steps:"
echo "1. Push to GitHub:"
echo "   git push -u origin main"
echo ""
echo "2. Enable GitHub features:"
echo "   - Go to your repository settings"
echo "   - Enable Issues and Discussions"
echo "   - Set up branch protection rules"
echo ""
echo "3. Update documentation:"
echo "   - Replace 'yourusername' with your actual GitHub username"
echo "   - Update email addresses in documentation"
echo "   - Customize project description"
echo ""
echo "4. Optional:"
echo "   - Set up GitHub Actions secrets"
echo "   - Configure repository topics"
echo "   - Add repository description"
echo ""

print_status "Repository setup completed! 🎉" 