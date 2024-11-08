# RRT Trial

This repository contains the Rapidly-Exploring Random Tree algorithm trial.

## Version History

### Version 1
- Basic implementation of the RRT algorithm.
- Initial functionality to search for a path from a start to a goal point.
- Simple grid setup without goal bias or obstacle avoidance.

### Version 2
- Added goal bias to increase the likelihood of sampling near the goal, improving convergence speed.
- Implemented collision checking to avoid obstacles during the path expansion process.
- Added bounds checking to ensure nodes stay within the grid limits.

### Version 3 (Final)
- Integrated real-time visualization of the RRT algorithm’s search process using Matplotlib.
- Start, goal, and random sample points are visually marked on the plot:
  - Start point in blue
  - Goal point in orange
  - Randomly sampled points in green
- Path progression and node connections are dynamically displayed in red.
- Refined collision and bounds checking to enhance robustness.

## Code Formatting and Linting

This repository follows Python style guidelines with automated formatting and linting:
- **Black**: Ensures consistent code style.
- **isort**: Organizes imports according to Python standards.
- **Flake8**: Identifies potential syntax errors and style inconsistencies.

### Pre-Commit Configuration

Used given `pre-commit-config.yaml` file. It is included to automate code checks before committing. The following tools are applied:
1. **Black**: For code formatting.
2. **isort**: For import sorting.
3. **Flake8**: For linting errors.

These checks were run on `rrt_trial.py` before uploading to GitHub to ensure code consistency.

