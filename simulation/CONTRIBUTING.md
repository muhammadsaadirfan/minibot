# Contributing to Mobile Robot Simulation

Thank you for your interest in contributing to the Mobile Robot Simulation project! This document provides guidelines and information for contributors.

## 🤝 How to Contribute

### Reporting Issues
- Use the GitHub issue tracker
- Provide detailed information about the problem
- Include system information (OS, ROS version, etc.)
- Attach relevant log files or screenshots

### Suggesting Enhancements
- Open a feature request issue
- Describe the enhancement clearly
- Explain the benefits and use cases
- Consider implementation complexity

### Code Contributions
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 🛠️ Development Setup

### Prerequisites
- Ubuntu 18.04 LTS
- ROS Melodic Desktop-Full
- Git
- Basic knowledge of ROS, C++, and Python

### Local Development
```bash
# Clone your fork
git clone https://github.com/yourusername/mobile-robot-simulation.git
cd mobile-robot-simulation

# Add upstream remote
git remote add upstream https://github.com/original-owner/mobile-robot-simulation.git

# Install dependencies
sudo apt update
sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
sudo apt install ros-melodic-controller-manager ros-melodic-diff-drive-controller
sudo apt install ros-melodic-joint-state-controller ros-melodic-position-controllers
sudo apt install ros-melodic-slam-gmapping ros-melodic-navigation
sudo apt install ros-melodic-map-server ros-melodic-amcl

# Build workspace
catkin_make
source devel/setup.bash
```

## 📝 Coding Standards

### C++ Code
- Follow ROS C++ Style Guide
- Use meaningful variable and function names
- Add comments for complex logic
- Include proper header guards
- Use const references for large objects

### Python Code
- Follow PEP 8 style guide
- Use meaningful variable names
- Add docstrings for functions and classes
- Handle exceptions appropriately

### ROS Specific
- Use proper ROS naming conventions
- Implement proper error handling
- Use ROS parameters for configuration
- Follow ROS package structure

## 🧪 Testing Guidelines

### Before Submitting
- Test your changes in simulation
- Verify all launch files work correctly
- Check that navigation still functions
- Ensure no regressions in existing functionality

### Testing Checklist
- [ ] Robot spawns correctly in Gazebo
- [ ] Controllers load without errors
- [ ] Navigation stack initializes properly
- [ ] SLAM mapping works as expected
- [ ] RViz visualization is correct
- [ ] No memory leaks or crashes

## 📋 Pull Request Process

### Creating a Pull Request
1. **Fork and clone** the repository
2. **Create a branch** for your feature/fix
3. **Make changes** following coding standards
4. **Test thoroughly** in simulation
5. **Commit changes** with clear messages
6. **Push to your fork**
7. **Create pull request**

### Pull Request Guidelines
- Use descriptive titles
- Provide detailed descriptions
- Reference related issues
- Include screenshots/videos if applicable
- Respond to review comments promptly

### Commit Message Format
```
type(scope): brief description

Detailed description if needed

Fixes #issue-number
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes
- `refactor`: Code refactoring
- `test`: Adding tests
- `chore`: Maintenance tasks

## 🏗️ Project Structure

### Adding New Features
- Place new launch files in appropriate `launch/` directories
- Add configuration files to `config/` directories
- Update URDF files in `urdf/` directories
- Document new features in README.md

### Package Organization
```
mobile_robot/
├── config/          # Controller and sensor configurations
├── launch/          # Launch files
├── map/            # Pre-built maps
├── rviz/           # RViz configurations
├── src/            # Source code
├── urdf/           # Robot descriptions
└── worlds/         # Gazebo worlds

navigation/
├── config/         # Navigation parameters
├── launch/         # Navigation launch files
└── rviz/          # Navigation RViz config
```

## 🔍 Code Review Process

### Review Criteria
- Code follows style guidelines
- Functionality works as expected
- No security vulnerabilities
- Proper error handling
- Adequate documentation
- Tests pass

### Review Process
1. Automated checks must pass
2. At least one maintainer review required
3. Address all review comments
4. Maintainers will merge when approved

## 📚 Documentation

### Code Documentation
- Add comments for complex algorithms
- Document function parameters and return values
- Include usage examples for new features
- Update README.md for significant changes

### API Documentation
- Document new ROS topics and services
- Update parameter descriptions
- Include launch file examples
- Provide troubleshooting guides

## 🐛 Bug Reports

### Required Information
- **Description**: Clear description of the bug
- **Steps to Reproduce**: Detailed reproduction steps
- **Expected Behavior**: What should happen
- **Actual Behavior**: What actually happens
- **Environment**: OS, ROS version, hardware
- **Logs**: Relevant error messages and logs

### Bug Report Template
```markdown
## Bug Description
Brief description of the issue

## Steps to Reproduce
1. Step 1
2. Step 2
3. Step 3

## Expected Behavior
What should happen

## Actual Behavior
What actually happens

## Environment
- OS: Ubuntu 18.04
- ROS Version: Melodic
- Hardware: (if applicable)

## Logs
Paste relevant logs here
```

## 🎯 Feature Requests

### Feature Request Guidelines
- Describe the feature clearly
- Explain the use case and benefits
- Consider implementation complexity
- Suggest possible approaches
- Check if similar features exist

## 📞 Getting Help

### Communication Channels
- GitHub Issues for bugs and feature requests
- GitHub Discussions for general questions
- Pull Request comments for code-related questions

### Before Asking for Help
- Check existing issues and documentation
- Search for similar problems
- Provide detailed information
- Include relevant code and logs

## 🙏 Recognition

### Contributors
- All contributors will be listed in the README
- Significant contributions will be highlighted
- Contributors will be added to the project's contributors list

### Types of Contributions
- Code contributions
- Documentation improvements
- Bug reports
- Feature suggestions
- Testing and feedback

## 📄 License

By contributing to this project, you agree that your contributions will be licensed under the MIT License.

---

Thank you for contributing to the Mobile Robot Simulation project! 🚀 