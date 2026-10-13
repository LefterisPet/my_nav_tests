# My Nav2 Tests

Μικρό και αυτόνομο πακέτο ROS 2 για αυτοματοποιημένο έλεγχο (Regression Testing) και συλλογή μετρικών από το Nav2 Stack.

## 📌 Απαιτήσεις
- ROS 2 (Humble / Jazzy)
- Navigation2 (Nav2) Stack
- TurtleBot3 Simulation Package

## 🔨 Build (Κατασκευή Πακέτου)
Για να χτίσεις το πακέτο μέσα στο workspace σου (`~/ros2_ws`):

```bash
cd ~/ros2_ws
colcon build --packages-select my_nav_tests
source install/setup.bash
